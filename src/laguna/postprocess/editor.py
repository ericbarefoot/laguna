"""Draw the ROI and include/exclude shapes on a plotted scan, in Jupyter.

Needs ipympl: run ``%matplotlib widget`` before creating a :class:`ShapeEditor`.
See ``examples/example_16_postprocess_roi_editor.ipynb``.
"""

from __future__ import annotations

import copy
from pathlib import Path

import ipywidgets as w
import matplotlib.pyplot as plt
import numpy as np
from IPython.display import display
from matplotlib.widgets import EllipseSelector, PolygonSelector, RectangleSelector

from .config import PostprocessConfig, save_postprocess_config
from .preview import draw_shapes, hillshade, scan_grid, show_dem
from .regions import Circle, Polygon, Rect

MODES = [
    "ROI rectangle",
    "Include polygon",
    "Exclude polygon",
    "Exclude rectangle",
    "Exclude circle",
]

HELP = """<b>Draw</b> with the left mouse button, then press <b>Add shape</b>.
Rectangle/circle: drag (circle drags out from its centre). Polygon: click vertices,
click the first vertex to close; <i>Esc</i> restarts. Pan/zoom with the toolbar
(turn it off again before drawing). <b>Remove last</b> removes the most recently
added include or exclude shape (whichever list the current mode edits).
<span style="color:#c90">ROI</span>, <span style="color:green">include</span>,
<span style="color:red">exclude</span>."""


def _r(v: float) -> float:
    return round(float(v), 1)


class ShapeEditor:
    """Edit ``cfg.roi``, ``cfg.include`` and ``cfg.exclude`` by drawing on a scan.

    The scan is drawn in ``cfg.frame`` coordinates, so set ``frame`` before
    drawing shapes. ``cfg`` is modified in place. **Save config** writes it to
    ``config_path``. **Preview DEM** runs the whole pipeline on this scan,
    writing to ``<out_path>/_preview``, and plots the result.

    Args:
        scan: Scan file to draw on. Shapes apply to every scan, so pick one
            with good coverage.
        cfg: Config to edit.
        config_path: YAML file to save to; leave as None to turn off saving.
        cell: Display grid cell size, mm.
    """

    def __init__(
        self,
        scan: str | Path,
        cfg: PostprocessConfig,
        config_path: str | Path | None = None,
        cell: float = 2.0,
    ):
        """Plot the scan and build the widgets."""
        self.scan = Path(scan)
        self.cfg = cfg
        self.config_path = Path(config_path) if config_path else None
        self.pending = None
        self._overlay = []

        grid, ext = scan_grid(self.scan, cell, transform=cfg.transform)
        with plt.ioff():
            self.fig, self.ax = plt.subplots(figsize=(8, 7.5))
        vmin, vmax = np.nanpercentile(grid, [2, 98])
        im = self.ax.imshow(
            grid, extent=ext, cmap="terrain", vmin=vmin, vmax=vmax, interpolation="nearest"
        )
        self.ax.imshow(
            hillshade(grid, cell), extent=ext, cmap="gray", alpha=0.35, interpolation="nearest"
        )
        self.fig.colorbar(im, ax=self.ax, label="z (mm)", shrink=0.8)
        self.ax.set_title(self.scan.name, fontsize=9)
        self.ax.set_aspect("equal")
        self.fig.canvas.header_visible = False

        props = dict(edgecolor="blue", linewidth=1.5, fill=False)
        self.selectors = {
            "rect": RectangleSelector(
                self.ax, self._on_rect, interactive=True, button=[1], props=props
            ),
            "circle": EllipseSelector(
                self.ax, self._on_circle, interactive=True, button=[1], props=props
            ),
            "poly": PolygonSelector(self.ax, self._on_poly, props=dict(color="blue", lw=1.5)),
        }
        self.selectors["circle"].add_state("square")
        self.selectors["circle"].add_state("center")

        self.mode = w.ToggleButtons(options=MODES, value=MODES[2])
        self.btn_add = w.Button(description="Add shape", button_style="success")
        self.btn_undo = w.Button(description="Remove last")
        self.btn_save = w.Button(
            description="Save config", button_style="primary", disabled=self.config_path is None
        )
        self.btn_preview = w.Button(
            description="Preview DEM",
            button_style="info",
            tooltip="Run the pipeline on this scan with the current shapes",
        )
        self.listing = w.HTML()
        self.out = w.Output()

        self.mode.observe(lambda _: self._activate(), names="value")
        self.btn_add.on_click(lambda _: self.add())
        self.btn_undo.on_click(lambda _: self.remove_last())
        self.btn_save.on_click(lambda _: self.save())
        self.btn_preview.on_click(lambda _: self.preview())
        self._activate()
        self.redraw()

    def _on_rect(self, eclick, erelease):
        x0, x1, y0, y1 = self.selectors["rect"].extents
        if x1 > x0 and y1 > y0:
            self.pending = Rect(_r(x0), _r(y0), _r(x1), _r(y1))

    def _on_circle(self, eclick, erelease):
        x0, x1, y0, y1 = self.selectors["circle"].extents
        if x1 > x0:
            self.pending = Circle(_r((x0 + x1) / 2), _r((y0 + y1) / 2), _r((x1 - x0) / 2))

    def _on_poly(self, verts):
        if len(verts) >= 3:
            self.pending = Polygon([(_r(x), _r(y)) for x, y in verts])

    def _activate(self):
        m = self.mode.value
        active = "circle" if "circle" in m else "poly" if "polygon" in m else "rect"
        for name, sel in self.selectors.items():
            sel.clear()
            sel.set_active(name == active)
            sel.set_visible(name == active)
        self.pending = None
        self.fig.canvas.draw_idle()

    def _target_list(self) -> list | None:
        m = self.mode.value
        if m.startswith("ROI"):
            return None
        return self.cfg.include if m.startswith("Include") else self.cfg.exclude

    def add(self):
        """Commit the shape just drawn to the current mode's list (or the ROI)."""
        if self.pending is None:
            self._msg("Nothing drawn yet.")
            return
        if self.mode.value.startswith("ROI"):
            self.cfg.roi = self.pending
        else:
            self._target_list().append(self.pending)
        self._msg(f"Added {self.pending}")
        self._activate()
        self.redraw()

    def remove_last(self):
        """Remove the most recent include or exclude shape, per the current mode."""
        lst = self._target_list()
        if lst:
            self._msg(f"Removed {lst.pop()}")
            self.redraw()

    def save(self):
        """Write the config to ``config_path``."""
        if self.config_path:
            save_postprocess_config(self.cfg, self.config_path)
            self._msg(f"Saved {self.config_path}")

    def preview(self):
        """Process this scan with the current shapes and plot the DEM."""
        from .pipeline import process_scan

        cfg = copy.deepcopy(self.cfg)
        cfg.output_dir = str(self.cfg.out_path / "_preview")
        with self.out:
            self.out.clear_output()
            print("processing ...")
            res = process_scan(self.scan, cfg, force=True)
            self.out.clear_output()
            if res.status != "ok":
                print(res.error)
                return
            print(
                f"{res.n_input} -> ROI {res.n_roi} -> shapes {res.n_shapes} -> denoised "
                f"{res.n_denoised} -> ground {res.n_ground}; "
                f"DEM nodata {res.dem_nodata_frac:.1%} of the kept area"
            )
            with plt.ioff():
                fig, ax = plt.subplots(figsize=(5, 7))
            show_dem(cfg.out_path / "dem" / f"{self.scan.stem}.tif", ax=ax, cfg=cfg)
            fig.canvas.header_visible = False
            display(fig.canvas)

    def redraw(self):
        """Redraw the shapes overlay and the shape listing from ``cfg``."""
        for a in self._overlay:
            a.remove()
        self._overlay = draw_shapes(self.ax, self.cfg)
        self.fig.canvas.draw_idle()
        rows = [f"<b>ROI</b>: {self.cfg.roi.to_dict()}"]
        rows += [
            f"<span style='color:green'>include {i}</span>: {s.to_dict()}"
            for i, s in enumerate(self.cfg.include)
        ]
        rows += [
            f"<span style='color:red'>exclude {i}</span>: {s.to_dict()}"
            for i, s in enumerate(self.cfg.exclude)
        ]
        self.listing.value = "<br>".join(rows)

    def _msg(self, text):
        with self.out:
            self.out.clear_output()
            print(text)

    def _ipython_display_(self):
        controls = w.HBox([self.btn_add, self.btn_undo, self.btn_save, self.btn_preview])
        display(
            w.VBox([w.HTML(HELP), self.mode, controls, self.fig.canvas, self.listing, self.out])
        )

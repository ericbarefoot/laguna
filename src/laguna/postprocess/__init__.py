"""Offline post-processing of saved scans: ROI, exclusion zones, denoise, ground filter, DEM.

Uses LAStools for the point-cloud work. Install the Python side with
``pip install 'laguna[postprocess]'``. See ``docs/guides/postprocess-scans.md``.
The interactive editor (:class:`laguna.postprocess.editor.ShapeEditor`) is
not imported here, because it needs a Jupyter kernel.
"""

from .config import (
    DemParams,
    GroundParams,
    NoiseParams,
    PostprocessConfig,
    load_postprocess_config,
    save_postprocess_config,
)
from .lastools import LastoolsError
from .pipeline import ScanResult, find_scans, is_current, output_paths, process_scan, run
from .regions import Circle, Polygon, Rect

__all__ = [
    "Circle",
    "DemParams",
    "GroundParams",
    "LastoolsError",
    "NoiseParams",
    "Polygon",
    "PostprocessConfig",
    "Rect",
    "ScanResult",
    "find_scans",
    "is_current",
    "load_postprocess_config",
    "output_paths",
    "process_scan",
    "run",
    "save_postprocess_config",
]

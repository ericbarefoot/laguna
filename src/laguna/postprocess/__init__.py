"""Offline post-processing of saved scans: ROI, exclusion zones, denoise, ground filter, DEM.

Uses LAStools for the point-cloud work. Install the Python side with
``pip install 'laguna[postprocess]'``. See ``docs/guides/postprocess-scans.md``.
The interactive editor (:class:`laguna.postprocess.editor.ShapeEditor`) is
not imported here, because it needs a Jupyter kernel.
"""

from .config import (
    ORIENT_MODES,
    DemParams,
    GroundParams,
    NoiseParams,
    PostprocessConfig,
    load_postprocess_config,
    save_postprocess_config,
)
from .lastools import LastoolsError
from .orient import oriented_points, scan_files, source_file
from .pipeline import ScanResult, find_scans, is_current, output_paths, process_scan, run
from .regions import Circle, Polygon, Rect

__all__ = [
    "ORIENT_MODES",
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
    "oriented_points",
    "output_paths",
    "process_scan",
    "run",
    "save_postprocess_config",
    "scan_files",
    "source_file",
]

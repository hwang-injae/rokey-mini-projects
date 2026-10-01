"""Resolve a YOLO weights path in a machine-independent way."""
import os

from ament_index_python.packages import get_package_share_directory


def resolve_model_path(model_path: str) -> str:
    """Return a usable path for `YOLO(...)`.

    - an existing file path -> used as-is
    - a bare filename (no '/') -> looked up in this package's ``models/`` dir
    - anything else -> returned unchanged (Ultralytics may still auto-download a
      known model name such as ``yolo11n.pt``)
    """
    if os.path.isfile(model_path):
        return model_path
    if '/' not in model_path:
        try:
            cand = os.path.join(
                get_package_share_directory('rc_tracker'), 'models', model_path)
            if os.path.isfile(cand):
                return cand
        except Exception:  # noqa: BLE001
            pass
    return model_path

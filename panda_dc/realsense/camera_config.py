import json
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class CameraConfig:
    multi_realsense: dict
    camera_overrides: dict
    visualizer: dict


def load_camera_config(path: Path) -> CameraConfig:
    if not path.exists():
        raise FileNotFoundError(f"Camera config not found: {path}")

    with path.open("r") as f:
        if path.suffix.lower() in {".yaml", ".yml"}:
            config = yaml.safe_load(f)
        elif path.suffix.lower() == ".json":
            config = json.load(f)
        else:
            raise ValueError(
                f"Unsupported camera config extension: {path.suffix}. "
                "Use .yaml, .yml, or .json."
            )

    if not config:
        raise ValueError(f"Camera config is empty: {path}")

    camera_entries = config.get("camera_defaults", config.get("cameras", []))
    camera_overrides = {}
    for i, camera in enumerate(camera_entries):
        if "serial_number" not in camera:
            raise ValueError(
                f"Camera config entry {i} is missing serial_number: {path}"
            )
        camera_overrides[str(camera["serial_number"])] = camera

    return CameraConfig(
        multi_realsense=config.get("multi_realsense", {}),
        camera_overrides=camera_overrides,
        visualizer=config.get("visualizer", {}),
    )


def get_realsense_kwargs(config: CameraConfig, record_fps: int) -> dict:
    settings = config.multi_realsense
    kwargs = {
        "record_fps": settings.get("record_fps", record_fps),
        "resolution": tuple(settings.get("resolution", [640, 480])),
        "depth_resolution": tuple(settings.get("depth_resolution", [640, 480])),
        "capture_fps": settings.get("capture_fps", 30),
        "enable_depth": settings.get("enable_depth", False),
        "enable_color": settings.get("enable_color", True),
        "enable_infrared": settings.get("enable_infrared", False),
        "record_depth": settings.get("record_depth", True),
    }

    serial_numbers = settings.get("serial_numbers")
    use_connected_cameras = settings.get(
        "use_connected_cameras", serial_numbers is None
    )
    if not use_connected_cameras:
        if not serial_numbers:
            raise ValueError(
                "multi_realsense.serial_numbers must be set when "
                "use_connected_cameras is false."
            )
        kwargs["serial_numbers"] = [str(serial) for serial in serial_numbers]
    elif serial_numbers:
        kwargs["serial_numbers"] = [str(serial) for serial in serial_numbers]

    return kwargs


def apply_camera_overrides(realsense, config: CameraConfig):
    for serial_number, camera in config.camera_overrides.items():
        if serial_number not in realsense.cameras:
            continue

        exposure = camera.get("exposure")
        gain = camera.get("gain")
        white_balance = camera.get("white_balance")

        if exposure is not None or gain is not None:
            realsense.cameras[serial_number].set_exposure(exposure=exposure, gain=gain)
        if white_balance is not None:
            realsense.cameras[serial_number].set_white_balance(white_balance)


def get_visible_serial_numbers(config: CameraConfig, all_serial_numbers: list[str]):
    visualizer = config.visualizer
    visible_serial_numbers = visualizer.get("visible_serial_numbers")
    hidden_serial_numbers = set(
        str(s) for s in visualizer.get("hidden_serial_numbers", [])
    )

    if visible_serial_numbers is not None:
        requested = [str(serial) for serial in visible_serial_numbers]
        return [serial for serial in requested if serial in all_serial_numbers]

    visible = []
    for serial in all_serial_numbers:
        camera = config.camera_overrides.get(serial, {})
        show = camera.get("show", camera.get("visible", True))
        if show and serial not in hidden_serial_numbers:
            visible.append(serial)
    return visible

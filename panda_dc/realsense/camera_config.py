from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class CameraConfig:
    camera_backend: str
    multi_realsense: dict
    multi_zed: dict
    camera_overrides: dict
    visualizer: dict


def load_config_file(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Config not found: {path}")

    with path.open("r") as f:
        if path.suffix.lower() in {".yaml", ".yml"}:
            config = yaml.safe_load(f)
        elif path.suffix.lower() == ".json":
            config = json.load(f)
        else:
            raise ValueError(f"Unsupported camera config extension: {path.suffix}. Use .yaml, .yml, or .json.")

    if not config:
        raise ValueError(f"Config is empty: {path}")
    if not isinstance(config, dict):
        raise ValueError(f"Config must contain a mapping: {path}")
    return config


def normalize_camera_defaults(camera_entries, path: Path) -> dict:
    if camera_entries is None:
        return {}

    camera_overrides = {}

    if isinstance(camera_entries, dict):
        for serial_number, camera in camera_entries.items():
            if camera is None:
                camera = {}
            if not isinstance(camera, dict):
                raise ValueError(f"Camera defaults for {serial_number} must be a mapping: {path}")
            camera_overrides[str(serial_number)] = {
                "serial_number": str(serial_number),
                **camera,
            }
        return camera_overrides

    for i, camera in enumerate(camera_entries):
        if not isinstance(camera, dict):
            raise ValueError(f"Camera config entry {i} must be a mapping: {path}")
        if "serial_number" not in camera:
            raise ValueError(f"Camera config entry {i} is missing serial_number: {path}")
        camera_overrides[str(camera["serial_number"])] = camera

    return camera_overrides


def get_camera_defaults(config: dict):
    if "camera_defaults" in config:
        return config["camera_defaults"]
    if "cameras" in config:
        return config["cameras"]
    if not any(key in config for key in ("camera_backend", "multi_realsense", "multi_zed", "visualizer")):
        return config
    return []


def load_camera_config(
    path: Path,
    camera_defaults_path: Optional[Path] = None,
) -> CameraConfig:
    config = load_config_file(path)
    camera_backend = config.get("camera_backend")
    if camera_backend is None:
        camera_backend = "zed" if "multi_zed" in config else "realsense"
    camera_backend = str(camera_backend).lower()
    if camera_backend not in {"realsense", "zed"}:
        raise ValueError(f"Unsupported camera_backend {camera_backend!r} in {path}. Use 'realsense' or 'zed'.")

    camera_overrides = {}
    if camera_defaults_path is not None:
        default_config = load_config_file(camera_defaults_path)
        camera_overrides.update(
            normalize_camera_defaults(
                get_camera_defaults(default_config),
                camera_defaults_path,
            )
        )

    camera_overrides.update(
        normalize_camera_defaults(
            get_camera_defaults(config),
            path,
        )
    )

    return CameraConfig(
        camera_backend=camera_backend,
        multi_realsense=config.get("multi_realsense", {}),
        multi_zed=config.get("multi_zed", {}),
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
        "flip_visual": get_flip_visual_config(settings, config.camera_overrides),
    }

    serial_numbers = settings.get("serial_numbers")
    use_connected_cameras = settings.get("use_connected_cameras", serial_numbers is None)
    if not use_connected_cameras:
        if not serial_numbers:
            raise ValueError("multi_realsense.serial_numbers must be set when use_connected_cameras is false.")
        kwargs["serial_numbers"] = [str(serial) for serial in serial_numbers]
    elif serial_numbers:
        kwargs["serial_numbers"] = [str(serial) for serial in serial_numbers]

    return kwargs


def get_zed_kwargs(config: CameraConfig, record_fps: int) -> dict:
    settings = config.multi_zed
    kwargs = {
        "record_fps": settings.get("record_fps", record_fps),
        "resolution": tuple(settings.get("resolution", [1280, 720])),
        "capture_fps": settings.get("capture_fps", 30),
        "put_fps": settings.get("put_fps"),
        "put_downsample": settings.get("put_downsample", True),
        "enable_depth": settings.get("enable_depth", False),
        "enable_color": settings.get("enable_color", True),
        "record_depth": settings.get("record_depth", True),
        "depth_mode": settings.get("depth_mode", "NEURAL"),
        "get_max_k": settings.get("get_max_k", 30),
        "init_parameters": settings.get("init_parameters"),
        "flip_visual": get_flip_visual_config(settings, config.camera_overrides),
        "verbose": settings.get("verbose", False),
    }

    serial_numbers = settings.get("serial_numbers")
    use_connected_cameras = settings.get("use_connected_cameras", serial_numbers is None)
    if not use_connected_cameras:
        if not serial_numbers:
            raise ValueError("multi_zed.serial_numbers must be set when use_connected_cameras is false.")
        kwargs["serial_numbers"] = [int(serial) for serial in serial_numbers]
    elif serial_numbers:
        kwargs["serial_numbers"] = [int(serial) for serial in serial_numbers]

    return kwargs


def get_flip_visual_config(settings: dict, camera_overrides: dict):
    camera_flip_visual = {
        str(serial_number): bool(camera["flip_visual"])
        for serial_number, camera in camera_overrides.items()
        if "flip_visual" in camera
    }

    if camera_flip_visual:
        flip_visual = settings.get("flip_visual")
        if isinstance(flip_visual, dict):
            return {**flip_visual, **camera_flip_visual}
        if flip_visual is None or flip_visual is False:
            return camera_flip_visual

    return settings.get("flip_visual", False)


def apply_camera_overrides(camera_system, config: CameraConfig):
    cameras_by_serial = {str(serial_number): camera for serial_number, camera in camera_system.cameras.items()}
    for serial_number, camera in config.camera_overrides.items():
        target = cameras_by_serial.get(str(serial_number))
        if target is None:
            continue

        exposure = camera.get("exposure")
        gain = camera.get("gain")
        white_balance = camera.get("white_balance")

        if exposure is not None or gain is not None:
            target.set_exposure(exposure=exposure, gain=gain)
        if white_balance is not None:
            target.set_white_balance(white_balance)


def get_visible_serial_numbers(config: CameraConfig, all_serial_numbers: list):
    visualizer = config.visualizer
    visible_serial_numbers = visualizer.get("visible_serial_numbers")
    hidden_serial_numbers = set(str(s) for s in visualizer.get("hidden_serial_numbers", []))
    serials_by_text = {str(serial): serial for serial in all_serial_numbers}

    if visible_serial_numbers is not None:
        requested = [str(serial) for serial in visible_serial_numbers]
        return [serials_by_text[serial] for serial in requested if serial in serials_by_text]

    visible = []
    for serial in all_serial_numbers:
        serial_text = str(serial)
        camera = config.camera_overrides.get(serial_text, {})
        show = camera.get("show", camera.get("visible", True))
        if show and serial_text not in hidden_serial_numbers:
            visible.append(serial)
    return visible

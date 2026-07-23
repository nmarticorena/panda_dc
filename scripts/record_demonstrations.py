from panda_dc.realsense.multi_realsense import MultiRealsense
from panda_dc.realsense.multi_camera_visualizer import MultiCameraVisualizer
from panda_dc.realsense.camera_config import (
    apply_camera_overrides,
    get_realsense_kwargs,
    get_visible_serial_numbers,
    get_zed_kwargs,
    load_camera_config,
)
from multiplezed import MultiCameraVisualizer as ZedMultiCameraVisualizer
from multiplezed import MultiZed
from panda_dc.teleoperation.teleop_cartesian_pandapy import Teleop
import json
import numpy as np
from pathlib import Path
import tyro
from dataclasses import dataclass
from datetime import datetime
import cv2
from InquirerPy.base.control import Choice
from InquirerPy import inquirer
from reactivex import operators as ops
from reactivex.subject import Subject
from typing import Optional

GELLO_CONFIG_DIR = Path(__file__).resolve().parents[1] / "config" / "gello_configs"


@dataclass
class Params:
    name: Optional[str] = None
    idx: Optional[int] = None
    data_dir: Path = Path("data")
    camera_config: Path = Path("config/default_cameras.yaml")
    robot_config: Path = Path("config/robots/default_robot.yaml")
    camera_defaults: Path = Path("config/camera_defaults.yaml")
    record_fps: int = 10


def get_experiment_name(name: Optional[str]) -> str:
    if name:
        return name

    return inquirer.text(
        message="Experiment name:",
        validate=lambda value: bool(value.strip()),
        invalid_message="Experiment name cannot be empty.",
        filter=lambda value: value.strip(),
    ).execute()


def select_gello_config() -> Path:
    gello_configs = sorted(
        GELLO_CONFIG_DIR.glob("*.json"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not gello_configs:
        raise FileNotFoundError(f"No GELLO configs found in {GELLO_CONFIG_DIR}")

    choices = [
        Choice(
            value=config,
            name=(
                f"{config.stem} "
                f"({datetime.fromtimestamp(config.stat().st_mtime).strftime('%d-%m-%Y')})"
            ),
        )
        for config in gello_configs
    ]

    return inquirer.select(
        message="Select GELLO configuration:",
        choices=choices,
    ).execute()


class DataRecorder:
    def __init__(self, params):
        self.params = params
        self.params.name = get_experiment_name(params.name)
        self.record_fps = params.record_fps
        self.camera_backend = None
        self.cams = None
        self.gui = None
        self.sensor_socket = None
        self.idx = params.idx if params.idx is not None else self.check_existing_demos()
        self.t = Teleop(gello_config=select_gello_config(), gripper = "robotiq")
        self.window_name = "Data Recorder"
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.demo_state_text = "Resetting..."
        self.demo_number_text = f"Demo Number: {self.idx}"
        self.record_data = False
        self.toggle_key = ord(" ")  # ASCII code for space bar
        self.discard_key = ord("d")  # ASCII code for 'd'
        self.key = None
        self.disposable = None
        self.states = []
        self.phase = 0.0
        self.gripper_action = 0.0

    @property
    def dataset_path(self):
        return self.params.data_dir / self.params.name

    def check_existing_demos(self):
        dataset_path = self.dataset_path
        if not dataset_path.exists():
            return 0
        episodes_path = dataset_path / "episodes"
        if not episodes_path.exists():
            return 0
        existing_demos = [
            int(p.name)
            for p in episodes_path.iterdir()
            if p.is_dir() and p.name.isdigit()
        ]
        if not existing_demos:
            return 0

        last_demo = max(existing_demos)
        redo_last_demo = inquirer.confirm(
            message=(
                f"Last demo is {str(last_demo).zfill(4)}. Do you want to redo it?"
            ),
            default=False,
        ).execute()
        return last_demo if redo_last_demo else last_demo + 1

    def record_state(self, s):
        record_frame = getattr(self.cams, "record_frame", None)
        if record_frame is not None:
            record_frame()
        state = {"gripper_action": self.gripper_action, **s}
        self.states.append(state)

    def setup_streams(self):
        self.teleop_state = Subject()
        self.t.set_callback(lambda x: self.teleop_state.on_next(x))
        self.record_stream = self.teleop_state.pipe(ops.sample(0.1))

    def setup(self):
        self.t.home_robot()

        camera_config = load_camera_config(
            self.params.camera_config,
            camera_defaults_path=self.params.camera_defaults,
        )
        self.camera_backend = camera_config.camera_backend
        if self.camera_backend == "zed":
            camera_kwargs = get_zed_kwargs(camera_config, self.record_fps)
            camera_class = MultiZed
            visualizer_class = ZedMultiCameraVisualizer
        else:
            camera_kwargs = get_realsense_kwargs(camera_config, self.record_fps)
            camera_class = MultiRealsense
            visualizer_class = MultiCameraVisualizer

        self.record_fps = camera_kwargs["record_fps"]
        self.cams = camera_class(**camera_kwargs)
        apply_camera_overrides(self.cams, camera_config)
        self.cams.start()
        self.setup_streams()

        visualizer_config = camera_config.visualizer
        visible_serial_numbers = get_visible_serial_numbers(
            camera_config, self.cams.serial_numbers
        )
        self.gui = visualizer_class(
            self.cams,
            visualizer_config.get("rows", 2),
            visualizer_config.get("cols", 2),
            visible_serial_numbers=visible_serial_numbers,
        )
        self.gui.start()

    def grasp(self, x):
        print(x)
        if x == "open":
            self.gripper_action = 0.0
            # self.t.gripper.grasp(1.0, 0.1, 60)
            self.t.gripper.open()
        else:
            self.gripper_action = 1.0
            # self.t.gripper.grasp(0.0, 0.1, 60)
            self.t.gripper.close()

    def toggle_record(self, discard=False):
        self.record_data = not self.record_data
        if self.record_data:
            # save a photo of the current state
            self.t.panda.start_controller(self.t.ctrl)
            self.demo_state_text = "Recording..."
            self.states = []
            episode_path = self.dataset_path / "episodes" / str(self.idx).zfill(4)
            video_path = episode_path / "video"
            video_path.mkdir(parents=True, exist_ok=True)
            self.cams.start_recording(str(video_path))
            self.disposable = self.record_stream.subscribe(
                lambda x: self.record_state(x)
            )
            print(f"Recording demonstration {self.idx} to {episode_path}")
        else:
            self.demo_state_text = "Resetting..."
            self.phase = 0.0
            self.t.send_home()
            if self.disposable:
                self.disposable.dispose()
                self.cams.stop_recording()
                wait_for_recording = getattr(
                    self.cams, "wait_for_recording_to_stop", None
                )
                if wait_for_recording is not None:
                    wait_for_recording()

                if not discard:
                    episode_path = (
                        self.dataset_path / "episodes" / str(self.idx).zfill(4)
                    )
                    with (episode_path / "state.json").open("w") as f:
                        json.dump(self.states, f, indent=4)
                    self.idx += 1
            print("Recording stopped.")
            print("Resetting...")

    def update_window(self):
        # Create a black background
        frame = np.zeros((600, 600, 3), dtype=np.uint8)

        # Set the color based on mode
        color = (0, 0, 255) if self.record_data else (0, 255, 0)
        cv2.rectangle(frame, (0, 0), (600, 600), color, -1)

        # Add text for demonstration state and number
        cv2.putText(
            frame,
            self.demo_state_text,
            (50, 180),
            self.font,
            1,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Demo Number: {self.idx}",
            (50, 250),
            self.font,
            1,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Phase: {self.phase}",
            (50, 320),
            self.font,
            1,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

        # Show the window
        cv2.imshow(self.window_name, frame)

    def start_recording(self):
        cv2.namedWindow(self.window_name)
        cv2.moveWindow(self.window_name, 600, 600)
        while True:
            self.update_window()
            self.key = cv2.waitKey(10) & 0xFF
            if self.key == self.toggle_key:
                self.toggle_record()
            elif self.key == self.discard_key:
                self.toggle_record(discard=True)
            elif self.key == ord("t"):  # transition to next phase
                self.phase += 1.0
            elif self.key == ord("q"):
                break

    def stop(self):
        if self.gui is not None:
            self.gui.stop(wait=True)
        if self.cams is not None:
            self.cams.stop(wait=True)
        self.t.relinquish()
        self.t.home_robot()
        cv2.destroyAllWindows()

    def run(self):
        try:
            self.setup()
            self.t.gello_gripper_stream.subscribe(lambda x: self.grasp(x))
            self.t.take_control_async()
            self.start_recording()
        finally:
            self.stop()


if __name__ == "__main__":
    params = tyro.cli(Params)
    data_recorder = DataRecorder(params)
    data_recorder.run()

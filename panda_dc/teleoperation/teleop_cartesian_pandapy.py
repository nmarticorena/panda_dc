import os
import numpy as np
from typing import Literal
import reactivex as rx
from reactivex import scheduler
from reactivex import operators as ops
from scipy.spatial.transform.rotation import Rotation as R
import pyrobotiqgripper as robotiq
import roboticstoolbox as rtb

import panda_py
from panda_dc.dynamixel.robot import DynamixelRobot
from panda_py import controllers
from panda_dc.teleoperation.gui import SwiftGui
from panda_dc.teleoperation.gripper import FrankaGripper, RobotiqGripper, NoneGripper

GripperType = Literal["none", "franka_hand", "robotiq"]


class Teleop:
    def __init__(
        self,
        hostname: str = "172.16.0.2",
        gripper: GripperType = "franka_hand",
        gui: bool = True,
        gello_config: str | os.PathLike | None = None,
    ):
        self.panda = panda_py.Panda(hostname)
        self.set_collision_behavior()
        if gripper == "franka_hand":
            self.gripper = FrankaGripper(hostname)
        elif gripper == "robotiq":
            self.gripper = RobotiqGripper()
        else:
            self.gripper = NoneGripper()
        self.gello = create_gello(gello_config)
        # self.home_q = np.deg2rad([-90, 0, 0, -90, 0, 90, 45])
        self.home_q = self.gello.robot_home
        self.stop_requested = False
        self._callback = None
        self.home_requested = False
        self.create_gello_streams()
        if gui:
            self.gui = SwiftGui()
        else:
            self.gui = None  # Replace with dummy api

    def set_callback(self, callback):
        self._callback = callback

    def set_collision_behavior(self):
        lower_torque_th_aceleration = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
        upper_torque_th_acceleration = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        lower_torque_th_nominal = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
        upper_torque_th_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        lower_force_th_acceleration = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        upper_force_th_acceleration = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        lower_force_th_nominal = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        upper_force_th_nominal = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
        self.panda.get_robot().set_collision_behavior(
            lower_torque_th_aceleration,
            upper_torque_th_acceleration,
            lower_torque_th_nominal,
            upper_torque_th_nominal,
            lower_force_th_acceleration,
            upper_force_th_acceleration,
            lower_force_th_nominal,
            upper_force_th_nominal,
        )


    def home_robot(self):
        self.panda.move_to_joint_position(self.home_q)

    def can_control(self) -> bool:
        gello_q = self.gello.get_joint_state()[:7]
        self.panda.get_robot().read_once()
        return check_joint_discrepency(gello_q, self.panda.q)

    def take_control(self):
        assert self.stop_requested == False
        if not self.can_control():
            while not self.can_control():
                self.gui.step(self.panda.q.tolist(), self.gello.get_joint_state()[:7])
            raise Exception("Gello and Panda are not in the same configuration")
        self.panda.move_to_joint_position(self.gello.get_joint_state()[:7])
        impedance = [400.0, 400.0, 400.0, 40.0, 40.0, 40.0]

        impedance = np.diag(impedance)
        self.ctrl = controllers.CartesianImpedance(
            impedance=impedance,
            nullspace_stiffness=0.2,
            damping_ratio=0.99,
            filter_coeff=0.9,
        )
        self.panda.enable_logging(1)
        self.panda_model = rtb.models.Panda()


        self.panda.start_controller(self.ctrl)

        print("---------YOU ARE IN CONTROL--------")
        with self.panda.create_context(frequency=200) as ctx:
            while ctx.ok() and not self.stop_requested:
                if self.home_requested:
                    self.home_robot()
                    self.home_requested = False

                gello_q = self.gello.get_joint_state()
                if isinstance(self.gripper, FrankaGripper):
                    pose = panda_py.fk(gello_q[:7])
                # if isinstance(self.gripper, RobotiqGripper):
                else: # Technically if we are not using any gripper this should be the eef
                    pose = self.panda_model.fkine(gello_q[:7], end = "panda_link8").A
                rot_mat = pose[:3, :3]
                quat = R.from_matrix(rot_mat).as_quat()
                self.ctrl.set_control(pose[:3, 3], quat)
                panda_log = self.panda.get_log()



                if self._callback:
                    while len(self.panda.get_log()["dq"]) < 1:
                        pass

                    panda_log = {k: v[0].tolist() for k, v in panda_log.items()}
                    x = {
                        "robot_q": self.panda.q.tolist(),
                        "robot_X_BE": np.array(self.panda.get_pose()).reshape(4,4).tolist(),
                        "gello_q": gello_q.tolist(),
                        "gripper_width": self.gripper.width_m(),
                        "libfranka": {**panda_log},
                        "gripper": {**self.gripper.read()},
                    }
                    self._callback(x)
                    self.gui.step(self.panda.q.tolist(), gello_q[:7])

        self.stop_requested = False
        print("--------RELINQUISHED CONTROL-------")

    def relinquish(self):
        self.stop_requested = True

    def send_home(self):
        self.home_requested = True

    def take_control_async(self):
        from threading import Thread

        self.thread = Thread(target=self.take_control)
        self.thread.start()

    def create_gello_streams(self, frequency=2.0):
        self.gello_joints_stream = (
            rx.interval(1.0 / frequency, scheduler=scheduler.NewThreadScheduler())
            .pipe(ops.map(lambda _: self.gello.get_joint_state()))
            .pipe(ops.map(lambda x: np.round(x[-1], 2)))
            .pipe(ops.distinct_until_changed())
            .pipe(ops.share())
        )

        threshold = 0.1
        self.gello_gripper_stream = (
            self.gello_joints_stream.pipe(ops.pairwise())
            .pipe(ops.filter(lambda x: np.abs(x[0] - x[1]) > threshold))
            .pipe(ops.map(lambda x: "open" if x[0] > x[1] else "close"))
            .pipe(ops.distinct_until_changed())
        )

        self.gello_button_stream = self.gello_gripper_stream.pipe(
            ops.filter(lambda x: x == "close")
        ).pipe(ops.map(lambda _: True))

def create_gello(config_path: str | os.PathLike | None = None) -> DynamixelRobot:
    import json
    import panda_dc
    import pathlib

    if config_path is None:
        panda_dc_dir = pathlib.Path(panda_dc.__path__[0]).parent
        config_path = panda_dc_dir / "config" / "gello_configs" / "grey.json"
    else:
        config_path = pathlib.Path(config_path)

    with config_path.open("r") as f:
        config = json.load(f)

    return DynamixelRobot(
        real=True,
        **config
    )


def check_joint_discrepency(q1, q2) -> bool:
    abs_deltas = np.abs(q1 - q2)
    id_max_joint_delta = np.argmax(abs_deltas)
    max_joint_delta = 0.8
    res = True
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print()
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            q1[id_mask],
            q2[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
            res = False
    return res


if __name__ == "__main__":
    teleop = Teleop()
    teleop.home_robot()
    teleop.gello_button_stream.subscribe(lambda _: teleop.relinquish())
    teleop.take_control()

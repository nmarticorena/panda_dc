import os
import pathlib
import numpy as np
from InquirerPy import inquirer
import panda_py
import json

from panda_dc.dynamixel.driver import DynamixelDriver

np.set_printoptions(precision=2, suppress=True)

robot = panda_py.Panda(os.environ["PANDA_IP"])
robot.move_to_joint_position([0.0, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0])

serial = inquirer.select(
    message="Select Gello serial port:",
    choices=os.listdir("/dev/serial/by-id/")
).execute()

gello = DynamixelDriver(
    port=f"/dev/serial/by-id/{serial}",
    baudrate=57600,
    ids=[1, 2, 3, 4, 5, 6, 7, 8]
)
current_robot_q = robot.q
gello_q = gello.get_joints()

offset = gello_q[:7] -  current_robot_q

gripper_open = gello_q[-1].copy()
confirm = inquirer.confirm(message="Close the gripper", default=True).execute()
gripper_close = gello.get_joints()[-1].copy()

diff = gripper_open - gripper_close
if abs(diff) < 0.01:
    exit("Gripper open and close positions are too close. Calibration failed.")


name = inquirer.text(message="Enter calibration name: ").execute()
calibration_data = {
    "joint_offsets": offset.tolist(),
    "port": f"/dev/serial/by-id/{serial}",
    "joint_signs": [1, 1, 1, 1, 1, 1, 1],
    "joint_ids": [1, 2, 3, 4, 5, 6, 7],
    "gripper_config": [8, gripper_open * 180 / np.pi, gripper_close * 180 / np.pi],
    "robot_home": current_robot_q.tolist(),
}
with open(f"config/gello_configs/{name}.json", "w") as f:
    json.dump(calibration_data, f, indent=4)

import os
import json

import panda_py
from panda_dc.src.dynamixel.robot import DynamixelRobot
from panda_dc.src.teleoperation.gui import SwiftGui
from InquirerPy import inquirer

name = inquirer.select(
    message="Select Gello configuration:",
    choices=[""] + [
        f[:-5] for f in os.listdir("config/gello_configs/") if f.endswith(".json")
    ],
).execute()


config = json.load(open(f"config/gello_configs/{name}.json", "r")) if name != "" else {}
print(config)

robot = panda_py.Panda(os.environ["PANDA_IP"])
gello = DynamixelRobot(
    real=True,
    **config,
)

gui = SwiftGui(robot.q, gello.get_joint_state()[:7])


print(robot.q)
while True:
    gui.step(robot.q, gello.get_joint_state()[:7])


     


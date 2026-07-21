'''
Script to check the parameters of the gripper
'''

import panda_desk
import trio
import json
from InquirerPy import inquirer


desk = panda_desk.Desk("172.16.0.2")

trio.run(desk.login,"franka","franka123")
trio.run(desk.take_control, True)
trio.run(desk.activate_fci)

config_name = inquirer.text(message="Enter the name of the configuration file (without .json):").execute()

params = trio.run(desk.get_eef_parameters)
with open(f"config/{config_name}.json", "w") as f:
    json.dump(params, f, indent = 4)

print("Please commit this configs if you want to use it in the future")

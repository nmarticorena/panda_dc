'''
Script to check the parameters of the gripper
'''

import panda_desk
import trio
import json


desk = panda_desk.Desk("172.16.0.2")

trio.run(desk.login,"franka","franka123")
trio.run(desk.take_control, True)
trio.run(desk.activate_fci)

params = trio.run(desk.get_eef_parameters)
with open("config/umi_gripper.json", "w") as f:
    json.dump(params, f, indent = 4)

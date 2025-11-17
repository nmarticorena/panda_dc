'''
Script to load the parameters of the gripper
'''

import panda_desk
import trio
import json


desk = panda_desk.Desk("172.16.0.2")

trio.run(desk.login,"franka","franka123")
trio.run(desk.take_control, True)
trio.run(desk.activate_fci)

params = json.load(open("config/umi_gripper.json"))
trio.run(desk.set_eef_parameters, params)
trio.run(desk.unlock)


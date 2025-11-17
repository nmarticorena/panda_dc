import json
import numpy as np

import spatialmath as sm
import spatialgeometry as sg
import roboticstoolbox as rtb
import swift
from panda_dc.src.utils import replace_gripper


env = swift.Swift()
env.launch()
robot = rtb.models.Panda()
robot.q = robot.qr

env.add(robot, robot_alpha=0.5)
replace_gripper(robot,env)
env.step()

params = json.load(open("config/umi_gripper.json"))

print(np.array(params["parameters"]["transformation"]).reshape(4,4))
transform = sm.SE3(np.array(params["parameters"]["transformation"]).reshape(4,4, order = "F"),check=False).norm()
print(transform)

tip = sg.Axes(0.1, pose = transform)
tip.attach_to(robot.links[-1])
env.add(tip)
print(params)

com = sg.Sphere(0.02, pose = sm.SE3(params["parameters"]["centerOfMass"]))
com.color = [1.0,0,0,1.0]
com.attach_to(robot.links[-1])
env.add(com)



env.hold()



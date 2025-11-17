from pathlib import Path

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import panda_dc

GRIPPER_PATHS = Path(panda_dc.__file__).parent.parent.joinpath("meshes")


def replace_gripper(robot: rtb.models.Panda, env: swift.Swift): # TODO: Improve for other grippers, at the moment is just the soft gripper
    gripper_path = GRIPPER_PATHS.joinpath("soft_gripper_finger.stl").__str__()

    alpha = env.swift_options[0]["robot_alpha"]

    left_finger = sg.Mesh(gripper_path, pose = sm.SE3(0.013,0,0.015) @ sm.SE3.Rz(90,unit= "deg") @ sm.SE3.Rx(180, unit = "deg"))
    left_finger.color = [0.,1.0,0,alpha]
    right_finger = sg.Mesh(gripper_path, pose = sm.SE3(-0.013,0,0.015) @ sm.SE3.Rz(-90,unit= "deg") @ sm.SE3.Rx(180, unit = "deg"))
    right_finger.color = [0.,1.0,0,alpha]
    
    env.add(left_finger)
    env.add(right_finger)

    left_finger.attach_to(robot.grippers[0].links[1])
    right_finger.attach_to(robot.grippers[0].links[2])

    return
        
if __name__ == "__main__":

    env = swift.Swift()
    env.launch()
    robot = rtb.models.Panda()
    robot.q = robot.qr

    env.add(robot, robot_alpha=0.5)
    replace_gripper(robot,env)
    env.step()

    env.hold()


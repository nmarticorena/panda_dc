from panda_py import libfranka
import os

gripper = libfranka.Gripper(os.environ.get("PANDA_IP"))

gripper.homing()
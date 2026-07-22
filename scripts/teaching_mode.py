'''
Script to test the teaching mode of the panda py
'''
import panda_py
import os

robot = panda_py.Panda(os.environ.get("PANDA_IP"), realtime_config=panda_py.libfranka.RealtimeConfig.kIgnore)
robot.teaching_mode(True)

input("Press Enter to continue...")

robot.teaching_mode(False)



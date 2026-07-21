import pyrobotiqgripper as rq
from InquirerPy import inquirer

#Create a Robotiq gripper object.
gripper = rq.RobotiqGripper()
gripper.activate()  # Activate the gripper

reply = inquirer.confirm("Do you want to close the gripper?").execute()
if reply:
    gripper.close()  # Close the gripper
    print("Gripper closed.")

reply = inquirer.confirm("Do you want to open the gripper?").execute()
if reply:
    gripper.open()  # Open the gripper
    print("Gripper opened.")

print("Gripper test completed.")




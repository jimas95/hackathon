from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
import time
# The robot object is what you use to control the robot
robot = InterbotixRobot(robot_name="px100", mrd=mrd)
mode = 'h'
# Let the user select the position


# robot.open_gripper(2.0)
# time.sleep(1)
# robot.close_gripper(2.0)
robot.go_to_home_pose()

robot.set_single_joint_position("wrist",-45)
robot.set_single_joint_position("wrist",45)


robot.set_single_joint_position("elbow",0.5)
# robot.set_single_joint_position("elbow",-1)


# robot.set_single_joint_position("shoulder",1)
# robot.set_single_joint_position("shoulder",-1)


print("------")
robot.go_to_home_pose()
print(robot.get_joint_positions())
robot.go_to_sleep_pose()

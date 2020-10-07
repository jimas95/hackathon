from interbotix_sdk.robot_manipulation import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
# The robot object is what you use to control the robot
robot = InterbotixRobot(robot_name="px100", mrd=mrd,gripper_pressure=1.0)
mode = 'h'
# Let the user select the position


robot.go_to_home_pose()
robot.go_to_sleep_pose()
robot.go_to_home_pose()
robot.set_gripper_pressure(1.3)
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit ")
    if mode == "h":
        robot.go_to_home_pose()
    elif mode == "s":
        robot.go_to_sleep_pose()
    elif mode == "c":
        robot.close_gripper(3.0)
    elif mode == "o":
        robot.open_gripper(3.0)
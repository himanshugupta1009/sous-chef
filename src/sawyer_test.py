import pybullet as p
import time
import math
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
from cairo_simulator.Simulator import Simulator
from cairo_simulator.Simulator import SimObject
from cairo_simulator.Simulator import ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer
from MotionPlanner import astar_path
from geometry_msgs.msg import Point, Pose

ball = None


def human_arm_controller(data):
    global ball
    p.resetBasePositionAndOrientation(bodyUniqueId = ball, posObj = [data.x, data.y, data.z], ornObj = [0,0,0,1])


def sawyer_arm_controller(data):
    pass


def main():
    global ball
    rospy.init_node("CAIRO_Sawyer_Simulator")

    #initialize environment
    use_real_time = True

    sim = Simulator() # Initialize the Simulator

    table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.9, 0, 0), (0, 0, 1.5708)) # intialize Table and rotate 90deg along z-axis
    sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8) #initialize sawyer
    ball_start_pos = (0.8, -1.5, 1.2)
    ball = p.loadURDF('sphere2red_nocol.urdf', basePosition=ball_start_pos, globalScaling=0.25) #ball representing a human hand
    sim_obj_1 = SimObject('cube0', 'cube_small.urdf', (0.8, 0.5, 0.55)) # place cube objects
    sim_obj_2 = SimObject('cube1', 'cube_small.urdf', (0.8, -0.5, 0.55))

    #initialize subscriber
    rospy.Subscriber("human_arm",Point,human_arm_controller)
    rospy.Subscriber("sawyer_arm_planner",Pose,sawyer_arm_controller)

    #create 3d matrix with obstacle objects
    obstacle = np.zeros((120,200,150))
    for i in range(0,180):
        y_curr = i
        x_curr = 80
        x_max = 110
        x_min = 50
        z_curr = math.floor(103.5 - 0.325*i)
        z_max = min(z_curr+30,150)
        z_min = max(z_curr-30,0)
        for j in range(x_min,x_max):
            for k in range(z_min,z_max):
                obstacle[j][i][k] = 1
    for k in range(0,55):
        for i in range(50, 120):
            for j in range(0,200):
                obstacle[i][j][k] = 1
    #get path
    start = list(p.getLinkState(sawyer_robot._simulator_id,sawyer_robot._end_effector_link_index)[0])
    start[0] = math.floor(start[0]*100)
    start[1] = math.floor((start[1]+1)*100)
    start[2] = math.floor(start[2]*100)
    end = [80, 150, 55]
    time.sleep(3)
    # sawyer_path = astar_path(obstacle,start,end)
    joint_config_1 = sawyer_robot.solve_inverse_kinematics([0.8,-0.5,1.5], [0,0,0,1])
    # sawyer_robot.move_to_joint_pos(joint_config_1)
    joint_config_2 = sawyer_robot.solve_inverse_kinematics([0.8,-0.5,0.65], [1,0,0,0])
    # sawyer_robot.move_to_joint_pos(joint_config_2)
    traj = ((2., joint_config_1), (4., joint_config_2))
    sawyer_robot.execute_trajectory(traj)
    # Loop until someone shuts us down
    while rospy.is_shutdown() is not True:
        sim.step()
    p.disconnect()


if __name__ == "__main__":
    main()

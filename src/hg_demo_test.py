import pybullet as p
import time
import rospy
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer
import numpy as np
import pdb
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


def main():
	rospy.init_node("CAIRO_Sawyer_Simulator")
	use_real_time = True

	sim = Simulator() # Initialize the Simulator

	# Add a table and a Sawyer robot
	table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.6, 0, 0), (0, 0, 1.5708))
	sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)
	robot_start_pos = p.getLinkState(sawyer_robot._simulator_id,sawyer_robot._end_effector_link_index)[0]

	#Add cubes and human (represented using a sphere)
	c1_start_pos =  (0.8,-0.7,0.55)
	c2_start_pos =  (0.8,0.7,0.55)
	ball_start_pos = (0.7, -1.5, 1.2)
	c1 = p.loadURDF('cube_small.urdf', basePosition=c1_start_pos)
	c2 = p.loadURDF('cube_small.urdf', basePosition=c2_start_pos)
	ball = p.loadURDF('sphere2red_nocol.urdf', basePosition=ball_start_pos, globalScaling=0.25)

	joint_config = sawyer_robot.solve_inverse_kinematics([1.0,0.9,1.5])
	sawyer_robot.move_to_joint_pos(joint_config)

	x_ball_positions = np.linspace(ball_start_pos[0],c2_start_pos[0],10)
	y_ball_positions  = np.linspace(ball_start_pos[1],c2_start_pos[1],10)
	z_ball_positions = np.linspace(ball_start_pos[2],c2_start_pos[2],10)

	x_robot_positions = np.linspace(robot_start_pos[0],c1_start_pos[0] - 0.05,10)
	y_robot_positions = np.linspace(robot_start_pos[0],c1_start_pos[1] - 0.05,10)
	z_robot_positions = np.linspace(robot_start_pos[0],c1_start_pos[2] - 0.05,10)

	for i in range(0,len(y_ball_positions)):
		p.resetBasePositionAndOrientation(bodyUniqueId = ball, posObj = [x_ball_positions[i], y_ball_positions[i], z_ball_positions[i]], ornObj = [0,0,0,1])
		joint_config = sawyer_robot.solve_inverse_kinematics([x_robot_positions[i],y_robot_positions[i],z_robot_positions[i]])
		sawyer_robot.move_to_joint_pos(joint_config)
		print(i)
		time.sleep(1)

if __name__ == "__main__":
    main()

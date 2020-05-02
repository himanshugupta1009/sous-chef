import time
import rospy
import numpy as np
from geometry_msgs.msg import Point

def main():
    rospy.init_node('human_arm',anonymous=False)
    pub = rospy.Publisher("human_arm",Point,queue_size=100)

    ball_start_pos = (0.7, -1.5, 1.2)
    c2_start_pos =  (0.75, 0.5, .55)

    x_ball_positions = np.linspace(ball_start_pos[0],c2_start_pos[0],500)
    y_ball_positions  = np.linspace(ball_start_pos[1],c2_start_pos[1],500)
    z_ball_positions = np.linspace(ball_start_pos[2],c2_start_pos[2],500)

    point = Point()
    for i in range(0,len(y_ball_positions)):
        point.x = x_ball_positions[i]
        point.y = y_ball_positions[i]
        point.z = z_ball_positions[i]
        pub.publish(point)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
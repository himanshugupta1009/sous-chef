import rospy
import copy 
import random
import numpy as np
from shapely.geometry import LineString, Polygon
from shapely.geometry import Point as Points
from shapely import affinity
import os
import datetime
from geometry_msgs.msg import Pose, Point, Quaternion
import pybullet as p
import time
import rospy
from cairo_simulator.Simulator import Simulator, SimObject, ASSETS_PATH
from cairo_simulator.Manipulators import Sawyer
import numpy as np
import pdb
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

workspace = np.array([[0.2,1.0],[-0.8,0.8],[0.6,1.4]])
graph = []
quat = Quaternion(0,0,0,1)
start_point_robot = Point(0.0,0.0,0.1)
generate_plot = True

##HARDCODED OBSTACLES
o = Polygon([(0.6,-1.6,1.3), (0.6,-1.6,1.1), (0.8,-1.6,1.3), (0.8,-1.6,1.1), \
    (0.6,0.8,1.3), (0.6,0.8,1.1), (0.8,0.8,1.3), (0.8,0.8,1.1)])
#o1 = Polygon([(-0.3,0.3), (-0.3,0.4), (-0.1,0.4), (-0.1,0.3)])
#obstacles = [affinity.scale(o,yfact=-1,origin=(0,0.35)),affinity.scale(o1,yfact=-1,origin=(0,0.35))]
obstacles = [affinity.scale(o,yfact=-1,origin=(0.7,-1.5,1.2))]


rospy.init_node("CAIRO_Sawyer_Simulator")
use_real_time = True

sim = Simulator() # Initialize the Simulator

# Add a table and a Sawyer robot
table = SimObject("Table", ASSETS_PATH + 'table.sdf', (0.6, 0, 0), (0, 0, 1.5708))
sawyer_robot = Sawyer("sawyer0", 0, 0, 0.8)

class Node:
    def __init__(self,pose,angles):
        self.pose = pose
        self.angles = np.array(angles)
        self.neighbours = []
        self.weight = []

    def distanceTo(self,newNode):
        return np.linalg.norm(newNode.angles - self.angles)

    def Link(self,index):
        self.neighbours.append(index)

    def eDist(self,newNode):
        ax=self.pose.position.x
        ay=self.pose.position.y
        az=self.pose.position.z
        bx=newNode.pose.position.x
        by=newNode.pose.position.y
        bz=newNode.pose.position.z
        return np.linalg.norm(np.array([ax,ay,az])-np.array([bx,by,bz]))


class Vertex:
    def __init__(self,node):
        self.f = 0.0
        self.g = 0.0
        self.h = 0.0
        self.previous = 0
        self.Node = node


def Nearest(newNode,NodeIndex):
    global graph
    closeAngIdx = []
    closeIdx = []
    angDist = .6
    euDist = .1
    for i in range(1,len(graph)):
        if i != NodeIndex and i not in graph[NodeIndex].neighbours and len(graph[NodeIndex].neighbours) < 5:
            if graph[i].eDist(newNode) < euDist:
                # angDist = graph[i].distanceTo(newNode)
                closeIdx.append(i)
    for i in closeIdx:
        if graph[i].distanceTo(newNode) < angDist:
            # euDist = graph[i].eDist(newNode)
            closeAngIdx.append(i)
    return closeAngIdx


def Exists(newNode):
    global graph
    exists = False
    for nodes in graph:
        if newNode.distanceTo(nodes) < 0.05:
            exists = True
            break
    return exists


def isInObstacle(newNode):
    x = float(newNode.pose.position.y)
    y = float(newNode.pose.position.x)
    z = float(newNode.pose.position.x)
    isIn = False
    for i in range(0,len(obstacles)):
        c = affinity.scale(obstacles[i],xfact=1.5,yfact=1.5)
        c = c.bounds
        if x > 0.6 and x < 0.8 and y > -1.6 and y < 0.8 and z> 1.1 and z<1.3:
            isIn = True
    return isIn


def get_position_and_orientation_from_Pose(some_given_pose):
    position = [some_given_pose.position.x, some_given_pose.position.y, some_given_pose.position.z]
    orientation = [some_given_pose.orientation.x, some_given_pose.orientation.y, some_given_pose.orientation.z, some_given_pose.orientation.w]
    return position, orientation


def GenerateRandom():
    global workspace,graph,quat

    x = (workspace[0][1]-workspace[0][0])*np.random.random_sample() + workspace[0][0]
    y = (workspace[1][1]-workspace[1][0])*np.random.random_sample() + workspace[1][0]
    z = (workspace[2][1]-workspace[2][0])*np.random.random_sample() + workspace[2][0]

    tPoint = Point(x,y,z)
    tPose = Pose()

    tPose.position = copy.deepcopy(tPoint)
    tPose.orientation = copy.deepcopy(quat)

    m,n = get_position_and_orientation_from_Pose(tPose)
    angles_limb = sawyer_robot.solve_inverse_kinematics(m,n)
    tNode = Node(tPose,angles_limb)

    if len(graph)>0:
        while Exists(tNode):
            tNode = GenerateRandom()
    return tNode


def PRM():
    global graph, quat, start_point_robot

    start = Pose()
    start.position = copy.deepcopy(start_point_robot)
    start.orientation = copy.deepcopy(quat)
    m,n = get_position_and_orientation_from_Pose(start)
    starting_configuration = sawyer_robot.solve_inverse_kinematics(m,n)        
    start = Node(start,starting_configuration)
    graph.append(start)

    for i in range(0,2000):
        print(i)
        XNew = GenerateRandom()
        while isInObstacle(XNew):
            XNew = GenerateRandom()
        graph.append(XNew)

    print("HG")
    for i in range(len(graph)):
        XNearestIdx = Nearest(graph[i],i)
        for j in XNearestIdx:
            graph[i].Link(j)
            graph[j].Link(i)
    #os.system('clear')
    global generate_plot
    if generate_plot:
        fig = plt.figure()
        for i in range(len(graph)):
            if i == 0:
                plt.scatter(graph[i].pose.position.x,graph[i].pose.position.y,c='g',marker='o')
            else:
                plt.scatter(graph[i].pose.position.x,graph[i].pose.position.y,c='r',marker='x')
            for j in range(len(graph[i].neighbours)):
                Lx = np.array([graph[i].pose.position.x,graph[graph[i].neighbours[j]].pose.position.x])
                Ly = np.array([graph[i].pose.position.y,graph[graph[i].neighbours[j]].pose.position.y])
                Lz = np.array([graph[i].pose.position.z,graph[graph[i].neighbours[j]].pose.position.z])
                plt.plot(Lx,Ly,c='k')
            for c in obstacles:
                y,x = c.exterior.xy
                plt.plot(x,y)
        fig.savefig('/home/himanshu/Desktop/plot.png')
        plt.show()



def reconstruct_path(current):
    total_path = [current]
    while total_path[-1] != total_path[-1].previous:
        current = current.previous
        total_path.append(current)
    for i in range(0,len(total_path)):
        total_path[i] = total_path[i].Node
    return total_path


def A_Star(start,goal):
    global graph
    graph2 = []
    for node in graph:
        graph2.append(Vertex(node))

    for i in range(0,len(graph2)):
        graph2[i].f = 1000
        graph2[i].g = 1000

    openSet = [graph2[start]]
    closedSet = []

    openSet[0].f = openSet[0].Node.eDist(graph2[goal].Node)
    openSet[0].g = 0
    openSet[0].previous = openSet[0]
    
    while len(openSet)>0:
        winner = 0
        
        for i in range(len(openSet)):
            if openSet[i].f < openSet[winner].f:
                winner = i
        current = openSet[winner]

        if current.Node.eDist(graph2[goal].Node) < 0.01:
            return reconstruct_path(current)
        
        openSet.pop(winner)
        closedSet.append(current)
        
        for ni in current.Node.neighbours:
            neighbour = graph2[ni]

            tentG = current.g + current.Node.eDist(neighbour.Node) 
            if not humanCost(current, neighbour): ##THIS IS THE EVALUATION OF WHETHER THE TRANSITION IS SAFE
                if tentG < neighbour.g:
                    neighbour.previous = current
                    neighbour.g = tentG
                    neighbour.f = neighbour.g + current.Node.eDist(graph2[goal].Node)
                    if neighbour not in openSet:
                        openSet.append(neighbour)
    closest = closedSet[-1]
    for i in range(0,len(closedSet)):
        if closedSet[i].Node.eDist(graph2[goal].Node)<closest.Node.eDist(graph2[goal].Node):
            closest = closedSet[i]
    return reconstruct_path(closest) 
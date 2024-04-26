#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
import math
import tf
import json
import csv

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

GT_PATH = '/home/harsh/catkin_ws/src/multi_husky/multi_husky/scripts/data2.csv'
class HuskyPathController:
    def __init__(self) -> None:
        rospy.init_node('pqm_driver_node', anonymous=True)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.gz_linkstate_callback)
        self.path_list = None
        self.curr_pose = (None,None)
        self.heading = None
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.reset_husky()
        rospy.Rate(1).sleep()


        # PARAMS
        self.rate_fps = 20 # Rate of the node
        self.turn_fps = 60 # Rate of the turn
        self.rotating_degree_threshold = 6 # Degree threshold to rotate only
        self.small_angle_difference = 0.3 # Radian difference above which turn will be proportional
        self.rotation_speed = 0.35 # Speed of rotation
        self.intermediate_goal_distance = 0.3 # Distance threshold to reach intermediate goal
        self.large_speed_threshold = 5 # Distance threshold above which large speed will be used
        self.large_linear_speed = 1.5 # Large speed
        self.small_linear_speed = 0.4 # Small speed
        self.driving_rotation_speed = 0.02 # Speed of rotation while driving to account for small turning

        self.rate = rospy.Rate(self.rate_fps)
        self.turn_fps = rospy.Rate(self.turn_fps)


    def reset_husky(self):
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        empty_twist =Twist()

        clear_input_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        clear_input_pub.publish(empty_twist)

        try:
            state_msg = ModelState()
            state_msg.model_name = 'husky'
            state_msg.pose.position.x =  0
            state_msg.pose.position.y = 0
            state_msg.pose.position.z =  0.2
            state_msg.pose.orientation.x = 0
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 1

            gazebo_resp = set_state(state_msg) 
        except rospy.ServiceException as e:
            print(e)


    # Add the XY Coordinates of the path to the list below
    def update_path(self,new_path_list=None):
        xy = []

        if new_path_list is None:
            xy.append([-0.5,29])
            xy.append([-70,29])
            xy.append([-0.5,29])
            xy.append([-0.5,93])
            xy.append([24,93])
            xy.append([-0.5,93])
            xy.append([-0.5,151.5])
            xy.append([27,151.5])
            xy.append([-0.5,151.5])
            xy.append([-50,151.5])
            xy.append([-103,151.5])
            xy.append([-41.5,151.5])
            xy.append([-41.5,215])
            xy.append([-89,215])
            xy.append([60,215])
            xy.append([6,215])
            xy.append([6,284])

        else:
            for p in new_path_list:
                xy.append(p)

        self.path_list = xy.copy()


    def gz_linkstate_callback(self,data):

        self.curr_pose = (data.pose[-5].position.x,data.pose[-5].position.y)

        quaternion = (data.pose[-5].orientation.x,data.pose[-5].orientation.y,data.pose[-5].orientation.z,data.pose[-5].orientation.w)
        r,p,y = tf.transformations.euler_from_quaternion(quaternion)

        self.heading = y

        # Saving data to CSV
        with open(GT_PATH, 'a', newline='') as csvfile:  # 'a' for appending, 'w' for writing (creates new file)
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([self.curr_pose[0], self.curr_pose[1], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])

    # Get direction of turn and magnitude difference between current heading and goal in [-pi,pi] range
    def get_direction(self,a):
        if a > 0 and self.heading < 0:
            if self.heading < -math.pi/2:
                dir = -1
                new_h = 2*math.pi-abs(self.heading)
                mag = abs(new_h-a)
            else:
                dir = 1
                mag = abs(a-self.heading)
        elif a < 0 and self.heading > 0:
            if a < -math.pi/2:
                dir = 1
                new_a = 2*math.pi-abs(a)
                mag = abs(new_a-self.heading)
            else:
                dir = -1
                mag = abs(self.heading-a)
        else:
            dir = 1 if (a-self.heading)>0 else -1
            mag = abs(a-self.heading)

        return dir,mag   

    def euc_dist(self,p1,p2):
        return math.sqrt((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)

    def run_controller(self,path_list=None):
        rotating_flag = False

        # Wait for the first pose to be received
        while self.curr_pose == (None,None):
            pass

        # Update the path list if a new one is provided
        if path_list is not None:
            self.update_path(path_list)

        # Check if path exists
        if self.path_list:
            path = self.path_list.copy()

            while not rospy.is_shutdown():
                print("Path Exists")
                try:
                    # Get intermediate goal
                    inter_goal = path.pop(0)
                    while self.euc_dist(self.curr_pose,inter_goal) > self.intermediate_goal_distance:

                        angle = math.atan2((inter_goal[1]-self.curr_pose[1]),(inter_goal[0]-self.curr_pose[0]))
                        twist_msg = Twist()
                        rot_dir,magnitude = self.get_direction(angle)
                        # Rotate until the angle difference is less than threshold
                        while magnitude>self.rotating_degree_threshold*math.pi/180:
                            print(f"Rotating: {'CCW' if rot_dir>0 else 'CW'},{magnitude}")
                            if not rotating_flag:
                                rotating_flag = True
                                rot_dir,magnitude = self.get_direction(angle)
                                if magnitude < self.small_angle_difference:
                                    twist_msg.angular.z = self.rotation_speed*rot_dir
                                else:
                                    twist_msg.angular.z = self.rotation_speed*rot_dir*magnitude
                            twist_msg.linear.x = 0
                            self.cmd_pub.publish(twist_msg)
                            # self.rate.sleep()
                            self.turn_fps.sleep()
                            angle = math.atan2((inter_goal[1]-self.curr_pose[1]),(inter_goal[0]-self.curr_pose[0]))
                            _,magnitude = self.get_direction(angle)
                        print("CURR X_Y:",round(self.curr_pose[0],4),round(self.curr_pose[1],4)," INTER. GOAL:",round(inter_goal[0],4),round(inter_goal[1],4)," DIST:",round(self.euc_dist(self.curr_pose,inter_goal),4))

                        rotating_flag = False
                        
                        angle = math.atan2((inter_goal[1]-self.curr_pose[1]),(inter_goal[0]-self.curr_pose[0]))

                        if self.euc_dist(self.curr_pose,inter_goal) > self.large_speed_threshold:
                            twist_msg.linear.x = self.large_linear_speed
                        else:
                            twist_msg.linear.x = self.small_linear_speed

                        rot_dir_2,mag_2 = self.get_direction(angle)
                        if mag_2 < self.driving_rotation_speed:
                            twist_msg.angular.z = 0.0
                        else:
                            twist_msg.angular.z = self.driving_rotation_speed*rot_dir_2

                        self.cmd_pub.publish(twist_msg)
                        self.rate.sleep()
                        

                except Exception as e:
                    print(e)
                    break

            print(f"Final Pose: {self.curr_pose}")
            rospy.loginfo("GOAL REACHED")
        else:
            rospy.loginfo("No Path Exists, Check the goal")
            exit(0)


        
if __name__ == '__main__':
    try:
        new_path = [[1,0],[2,0],[2,-1],[2,-2]]

        path_json = None
        with open('/home/harsh/catkin_ws/src/multi_husky/multi_husky/scripts/gz_path_list.json') as f:
            path_json = json.load(f)

        new_path = path_json['warehouse']

        controller = HuskyPathController()
        controller.update_path(new_path) # Uncomment this line to use a new path
        # controller.update_path()
        controller.run_controller()

    except rospy.ROSInterruptException:
        pass

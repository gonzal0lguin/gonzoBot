#!/usr/bin/env python3
"""
This is the main script used for the EL5206 Robotics component. This script is
written Python 3 and defines a EL5206_Robot node for you to program. You will
find notes on how the code works, auxiliary functions and some for you to 
program.

Authors: Your dear 2022.Spring.teaching_staff
(Eduardo Jorquera, Ignacio Dassori & Felipe Vergara)
"""
import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
import sys
import tf
import tf2_ros
import time
import yaml
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from math import atan2, sqrt

class EL5206_Robot:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('EL5206_Main_Node', anonymous=False)

        # Attributes
        self.robot_frame_id = 'base_link'
        self.odom_frame_id  = 'odom'
        self.currentScan =  None
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0
        self.gt_x     = 0.0
        self.gt_y     = 0.0
        self.gt_yaw   = 0.0
        self.odom_lst = []
        self.gt_lst   = []
        self.poses_to_save = 100 
        self.target_x   = 0.0
        self.target_y   = 0.0
        self.target_yaw = 0.0
        self.target_list = []
        self.target_reached_pose = [] # [xodom, yodom, todom, xgt, ygt, tgt]
        self.path = rospkg.RosPack().get_path('gonzo_gazebo')

        self.last_error_linear = 0
        self.last_error_angular = 0
        self.accum_error_linear = 0
        self.accum_error_angular = 0

        self.Kd_linear = 0.3
        self.Kd_angular = 0.1
        self.Kp_linear = 1.0
        self.Kp_angular = 0.8
        self.Ki_linear = 0.8
        self.Ki_angular = 1.0

        self._control_rate = rospy.Rate(10)

        self.speed_cmd = Twist()

        # Extra variable to print odometry
        self.odom_i = 0
        self.target_reached = False

        # Subscribers
        rospy.Subscriber("/mobile_base_controller/odom",Odometry,    self.odometryCallback)
        rospy.Subscriber("/ground_truth/state",    Odometry,    self.groundTruthCallback)
        rospy.Subscriber("/gonzo/sensors/lidar/scan",     LaserScan,   self.scanCallback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.poseCallback)

        # Publishers
        self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=5)

        # Timer
        self.update_timer = rospy.Timer( rospy.Duration(.5), self.timerCallback )

        # Parameters
        self.assignment = 2#rospy.get_param("assignment")
        self.nav_mode   = 2#rospy.get_param("nav_mode")



    """
    Callback functions are executed every time a ROS messaage is received by the
    Subscirbers defined in the __init__ method. You have to be careful with 
    these methods, because they are executed several times a second and should 
    be quick enough to finnish before the next message arrives. 
    
    Be careful with the variables stored in a callback, because these methods 
    can be threaded, they may change some of the variables you are using in the 
    main code.
    """
    def scanCallback(self, msg):
        """
        Receives a LaserScan message and saves it self.currentScan.
        """
        self.currentScan = msg


    def odometryCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates and save the in the self.odom_x, 
        self.odom_y and self.odom_yaw attributes.
        """
        self.odom_i += 1
        #if self.odom_i%30==0:
            # Print one every 30 msgs
            #print("This is the Odometry message:")
            #print(msg)
        self.odom_x, self.odom_y, self.odom_yaw = self.odom2Coords(msg)
    

    def groundTruthCallback(self, msg):
        """
        Receives an Odometry message. Uses the auxiliary method odom2Coords()
        to get the (x,y,yaw) coordinates ans saves the in the self.gt_x, 
        self.gt_y and self.gt_yaw attributes
        """
        self.gt_x, self.gt_y, self.gt_yaw = self.odom2Coords(msg)


    def poseCallback(self, msg):
        """
        This method is activated whenever a message in the /target_pose topic 
        is published. For the assignments that require a target pose, you can 
        call your assignment functions from here or from the __main__ section
        at the end.

        Hint: in your terminal you can use the following command to send a pose
        of x,y,theta coordinates of 1, 2 and 3 respectively.
        $ rostopic pub /target_pose geometry_msgs/Pose2D '1.0' '2.0' '3.0'
        """
        # START: YOUR CODE HERE
        quat = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.target_x   = msg.pose.position.x
        self.target_y   = msg.pose.position.y
        self.target_yaw = yaw
        self.target_list.append([self.target_x, self.target_y, self.target_yaw])
        self.target_reached = False

        # END: YOUR CODE HERE


    def timerCallback(self,event):
        """
        This timer function will save the odometry and Ground Truth values
        for the position in the self.odom_lst and self.gt_lst so you can
        compare them afterwards. It is updated every 1 second for 300 poses 
        (about 5 mins).
        """
        if self.odom_x is not None and self.gt_x is not None and len(self.odom_lst)<self.poses_to_save:
            self.odom_lst.append( (self.odom_x, self.odom_y) )
            self.gt_lst.append( (self.gt_x, self.gt_y) )
                

    """
    Now let's define some auxiliary methods. These are used to solve the 
    problems in the assignments.
    """
    def odom2Coords(self, odom_msg):
        """
        This is an auxiliary method used to get the (x, y, yaw) coordinates from 
        an Odometry message. You can use the cmd "$ rostopic echo -n 1 /odom"
        in the terminal to check out the Odometry message attributes.

        Hint: Check the built-in functions in the tf.transformations package to
        see if there is one that handles the euler-quaternion transformation.
        http://docs.ros.org/en/melodic/api/tf/html/python/transformations.html
        """

        # START: YOUR CODE HERE
        quat = odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # END: YOUR CODE HERE
        return (x, y, yaw)
    

    def saveLaser(self): 
        """
        For the RANSAC experience, it is very common for students to save the
        Laser array and work form home with Jupyter Notebook or Google Colab.
        """
        # Wait for laser to arrive
        while self.currentScan is None:
            pass
        
        ranges = np.array(self.currentScan.ranges)
        a_min  = self.currentScan.angle_min
        a_inc  = self.currentScan.angle_increment
        angles = np.array([a_min + i*a_inc for i in range(len(ranges))])

        array_to_save = np.stack([ranges, angles], axis=1)
        with open(self.path+"/results/laser_ranges.npy", "wb") as f:
            np.save(f, array_to_save)
            print("Saved array of shape (%i, %i)"%array_to_save.shape)
            print("Look it up in the %s/results directory"%self.path)
    

    def plotOdomVsGroundTruth(self):
        """
        Imports a map image and plots the trajectory of the robot according to 
        the Odometry frame and Gazebo's Ground Truth.
        """
        if len(self.odom_lst)>0:
            img = plt.imread(self.path+'/maps/map.pgm')

            print('Image imported')
            # Import map YAML (This is a text file with information about the map)
            with open(self.path+"/maps/map.yaml", 'r') as stream:
                data       = yaml.safe_load(stream)
                origin     = data['origin']
                resolution = data['resolution']
                height     = img.shape[0]
            
            odom_arr = np.array(self.odom_lst)
            gt_arr   = np.array(self.gt_lst)
            
            odom_x_px = ((odom_arr[:,0] - 1.2 - origin[0])/resolution).astype(int)
            odom_y_px = (height-1 +(origin[1]-2.5-odom_arr[:,1])/resolution).astype(int)
            gt_x_px = ((gt_arr[:,0] - 1.2 - origin[0])/resolution).astype(int)
            gt_y_px = (height-1+ (origin[1]-2.5-gt_arr[:,1])/resolution).astype(int)

            plt.imshow(img,cmap='gray')
            plt.plot(odom_x_px, odom_y_px, color="red", linewidth=1, label='Odometry')
            plt.plot(gt_x_px, gt_y_px, color="blue", linewidth=1, label='Ground Truth')
            plt.legend()
            plt.title('Trajectory of the Robot')
            plt.axis('off')
            plt.savefig(self.path+'/results/trajectories.png')

    def plotOdomVsGroundTruth_ass2(self):
        """
        Imports a map image and plots the trajectory of the robot according to
        the Odometry frame and Gazebo's Ground Truth.
        """
        if len(self.odom_lst) > 0:
            img = plt.imread(self.path+'/maps/map.pgm')
            print('Image imported')
            # Import map YAML (This is a text file with information about the map)
            with open(self.path+"/maps/map.yaml", 'r') as stream:
                data = yaml.safe_load(stream)
                origin = data['origin']
                resolution = data['resolution']
                height = img.shape[0]

                odom_arr = np.array(self.odom_lst)
                gt_arr = np.array(self.gt_lst)
                target_arr = np.array(self.target_list)
                final_poses_odom = np.array([x['odom'] for x in self.target_reached_pose])
                final_poses_gt = np.array([x['gt'] for x in self.target_reached_pose])

                odom_x_px = ((odom_arr[:,0] - origin[0])/resolution)#.astype(int)
                odom_y_px = (height-1+ (origin[1]-odom_arr[:,1])/resolution)#.astype(int)
                gt_x_px = ((gt_arr[:,0] - origin[0])/resolution)#.astype(int)
                gt_y_px = (height-1+ (origin[1]-gt_arr[:,1])/resolution)#.astype(int)
                trgt_x = ((target_arr[:,0] - origin[0])/resolution)
                trgt_y = (height-1+ (origin[1]-target_arr[:,1])/resolution)
                final_pose_gt_x = ((final_poses_gt[:,0] - origin[0])/resolution)
                final_pose_gt_y = (height-1+ (origin[1]-final_poses_gt[:,1])/resolution)
                final_pose_odom_x = ((final_poses_odom[:,0] - origin[0])/resolution)
                final_pose_odom_y = (height-1+ (origin[1]-final_poses_odom[:,1])/resolution)

                plt.plot(odom_x_px, odom_y_px, color="red", linewidth=1, label='Odometry')
                plt.plot(gt_x_px, gt_y_px, color="blue", linewidth=1, label='Ground Truth')
                if len(self.target_list) > 0:
                    plt.annotate('Start', (gt_x_px[0], gt_y_px[0]), fontsize=12, va='top')
                    plt.scatter(trgt_x, trgt_y, color="black", marker="x")
                    plt.scatter(final_pose_gt_x, final_pose_gt_y, color="blue", marker="x")
                    plt.scatter(final_pose_odom_x, final_pose_odom_y, color="red", marker="x")
                    plt.legend()
                    plt.title('Trajectory of the Robot')
                    plt.axis('off')
                    plt.grid()
                    plt.savefig(self.path+'/results/as2plot.png')

    def printOdomvsGroundTruth(self):
        """
        Prints the robot odometry and ground truth position/angle.
        """
        if self.odom_x is not None and self.gt_x is not None:
            print("                  Odometry         -        GroundTruth")
            print("(x,y,yaw):  (%6.2f,%6.2f,%6.2f) - (%6.2f,%6.2f,%6.2f)"%(self.odom_x,self.odom_y,self.odom_yaw,self.gt_x,self.gt_y,self.gt_yaw))


    def dance(self, timeout=300):
        """
        Demo function. Moves the robot with the vel_pub Publisher.
        """
        start_time = time.time()
        while time.time() - start_time < timeout and not rospy.is_shutdown():
            # Move forward
            twist_msg = Twist()
            twist_msg.linear.x  = 0.2
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Move backward
            twist_msg = Twist()
            twist_msg.linear.x  = -0.2
            twist_msg.angular.z = 0.0
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Turn left
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = 0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(1)

            # Turn right
            twist_msg = Twist()
            twist_msg.linear.x  = 0.0
            twist_msg.angular.z = -0.2
            self.vel_pub.publish(twist_msg)
            time.sleep(1)


    def target_tolerance(self, tol_lin=1e-2, tol_ang=0.2):
        x_tol = abs(self.target_x - self.odom_x) < tol_lin
        y_tol = abs(self.target_y - self.odom_y) < tol_lin
        theta_tol = abs(self.target_yaw - self.odom_yaw) < tol_ang

        return x_tol & y_tol & theta_tol 

    def save_target_reached_pose(self):
        pose = {'odom': [self.odom_x, self.odom_y, self.odom_yaw], 
                'gt': [self.gt_x, self.gt_y, self.gt_yaw]}

        self.target_reached_pose.append(pose)

    def orient_to_goal(self, ang_tol=.05):
        ang_diff = self.target_yaw - self.odom_yaw
        while abs(ang_diff) > ang_tol:
            self.speed_cmd.angular.z = ang_diff * self.Kp_angular
            self.vel_pub.publish(self.speed_cmd) 
            ang_diff = self.target_yaw - self.odom_yaw
        
        self.speed_cmd.angular.z = 0.0
        self.vel_pub.publish(self.speed_cmd) 

    def align_and_naivigate(self, lin_tol=.1, ang_tol=.02):
        if not self.target_reached:
            theta_0 = atan2(self.target_y-self.odom_y, self.target_x-self.odom_x)
            ang_diff = theta_0 - self.odom_yaw 
            lin_diff = sqrt((self.odom_x-self.target_x)**2 + (self.odom_y-self.target_y)**2)

            # Angular movement
            while abs(ang_diff) > ang_tol:
                self.speed_cmd.angular.z = max(-0.5, min(ang_diff * self.Kp_angular, 0.5))
                self.vel_pub.publish(self.speed_cmd) 
                ang_diff = theta_0 - self.odom_yaw
                print('odom: ', self.odom_yaw)
                self._control_rate.sleep()

            self.speed_cmd.angular.z = 0.0
            self.vel_pub.publish(self.speed_cmd) 
            rospy.loginfo_once('Angular target reached, starting linear movement')
            
            time.sleep(0.5)
            
            # Linear movement
            accel_prof = 0.1
            while abs(lin_diff) > lin_tol:
                self.speed_cmd.angular.z = 0.0
                self.speed_cmd.linear.x = max(0, min(lin_diff * self.Kp_linear, 0.3)) * accel_prof
                lin_diff = sqrt((self.odom_x-self.target_x)**2 + (self.odom_y-self.target_y)**2)
                
                self.vel_pub.publish(self.speed_cmd) 
                self._control_rate.sleep()
                accel_prof += 0.02
                if accel_prof >= 1.0: accel_prof = 1.0
                
            self.speed_cmd.linear.x = 0.0
            self.vel_pub.publish(self.speed_cmd) 
            self.orient_to_goal()

            rospy.loginfo_once('Target Reached!')
            self.target_reached = True

        self.save_target_reached_pose()
        rospy.loginfo_once('Target Reached!')

    def navigate_pid(self):
        if not self.target_reached:
            ang_error = atan2(self.target_y-self.odom_y, self.target_x-self.odom_x) - self.odom_yaw 
            lin_error = sqrt((self.odom_x-self.target_x)**2 + (self.odom_y-self.target_y)**2)

            accel_factor = 0.2
            while True:
    
                self.accum_error_linear += lin_error
                self.accum_error_angular += ang_error
                
                # Update last error
                self.last_error_linear = lin_error
                self.last_error_angular = ang_error
                
                dt = 1 / 10.0

                delta_error_linear =  self.last_error_linear / dt
                delta_error_angular = self.last_error_angular / dt
                accum_error_linear =  self.last_error_linear * dt
                accum_error_angular = self.last_error_angular * dt

                # Calculate control commands using PD controller
                linear_speed = self.Kp_linear * lin_error + \
                               self.Kd_linear * delta_error_linear +\
                               self.Ki_linear * accum_error_linear

                angular_speed = self.Kp_angular * ang_error +\
                                self.Kd_angular * delta_error_angular +\
                                self.Ki_angular * accum_error_angular

                ang_error = atan2(self.target_y-self.odom_y, self.target_x-self.odom_x) - self.odom_yaw 
                lin_error = sqrt((self.odom_x-self.target_x)**2 + (self.odom_y-self.target_y)**2)

                # Publish control commands
                self.speed_cmd.linear.x = max(0, min(linear_speed, 0.3)) * accel_factor
                self.speed_cmd.angular.z = max(-1, min(angular_speed, 1))

                self.vel_pub.publish(self.speed_cmd)
                self._control_rate.sleep()

                self._pid_timer = rospy.Time.now() # update timer

                accel_factor += 0.1
                if accel_factor > 1.0: accel_factor = 1.0
                if abs(lin_error) < 0.05: break
            
            self.speed_cmd.linear.x  = 0.0
            self.speed_cmd.angular.z = 0.0
            self.vel_pub.publish(self.speed_cmd)
            self.orient_to_goal()

            self.target_reached = True
        
        self.save_target_reached_pose()
        rospy.loginfo_once('Target Reached!')

    ##########################################
    ###### ASSIGNMENT 4 FUNCTIONS ############
    ##########################################

    def assignment_1(self):
        # You can use this method to solve the Assignment 1.
        if len(self.odom_lst) > 100:
            self.printOdomvsGroundTruth()
            self.plotOdomVsGroundTruth()


    def assignment_2(self):
        # You can use this method to solve the Assignment 2.
        if self.target_reached:
            rospy.loginfo_once('Waiting for target!')
        else:
            if self.nav_mode == 1:
                self.align_and_naivigate()

            elif self.nav_mode == 2:
                self.navigate_pid()
            
            else:
                rospy.loginfo_once('Incorrect nav mode')
                rospy.signal_shutdown()
                    

        # END: YOUR CODE HERE
        rospy.loginfo_once('started node')


    def assignment_3(self):
        # You can use this method to solve the Assignment 3.
        # START: YOUR CODE HERE

        # END: YOUR CODE HERE
        pass

    
    def cal_potential():
        pass
    
    def assignment_4(self):
        # You can use this method to solve the Assignment 4.
        
        pass




if __name__ == '__main__':
    node = EL5206_Robot()
    print("EL5206 Node started!")

    try:
        # Demo function
        while not rospy.is_shutdown():

            if node.assignment == 1:
                node.assignment_1()

            elif node.assignment == 2:
                node.assignment_2()
                #if len(node.target_list) > 3: # plot after 4 commanded poses
                    #node.plotOdomVsGroundTruth_ass2()
                    #print(node.target_reached_pose)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")

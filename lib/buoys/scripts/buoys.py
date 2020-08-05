#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped
from vanttec_uuv.msg import GuidanceWaypoints
from usv_perception.msg import obj_detected, obj_detected_list
from nav_msgs.msg import Path

# Class Definition
class AutoNav:
    def __init__(self):
        self.ned_x = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.target_x = 0
        self.target_y = 0
        self.ned_alpha = 0
        self.distance_away = 5
        self.waypoints = GuidanceWaypoints()
        self.uuv_path = Path()
        self.PI = 3.1416
        self.waypoints_second_buoy = []
        self.waypoints_first_buoy = []
        self.radius = 1
        self.selection = -1
        self.long_sub = 0.225
        self.x_farther = 0
        self.y_farther = 0
        self.activation = True
        #Waypoint test instead of perception node


        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber("/uuv_planning/motion_planning/desired_trajectory1", Int32, self.desired_callback)
        '''
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        '''

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

        #Waypoint test instead of perception node

        self.objects_list = [
            {
                'X': 2,
                'Y': -2,
                'Z': 0
            },
            {
                'X': 2,
                'Y': 2,
                'Z': 0
            }    
        ]
    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z
    
    def desired_callback(self, data):
        self.selection = data.data

    def calculate_two_bouys(self):
        '''
        @name: calculate_two_bouys
        @brief: Returns two waypoints as desired positions. The first waypoint is
          between the middle of the gate and it right or left post, and the second a distance to the front 
        @param: --
        @return: --
        '''
        if(self.activation == True):
            x_list = []
            y_list = []
            distance_list = []
            for i in range(len(self.objects_list)):
                x_list.append(self.objects_list[i]['X'])
                y_list.append(self.objects_list[i]['Y'])
                distance_list.append(math.pow(x_list[i]**2 + y_list[i]**2, 0.5))

            ind_g1 = np.argsort(distance_list)[0]
            ind_g2 = np.argsort(distance_list)[1]

            #Calculate distance away from first buoy

            x1 = x_list[ind_g1]
            y1 = -1*y_list[ind_g1]

            alpha1 = math.atan2(y1,x1) + math.pi/2
            if (abs(alpha1) > (math.pi)):
                alpha1 = (alpha1/abs(alpha1))*(abs(alpha1) - 2*math.pi)

            self.ned_alpha = alpha1 + self.yaw
            if (abs(self.ned_alpha) > (math.pi)):
                self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)
                    
            xm1, ym1 = self.gate_to_body(0,0,alpha1,self.x_farther+x1,self.y_farther+y1)
            self.waypoints_first_buoy = [self.x_farther+x1,self.y_farther+y1,xm1,ym1]

            #Calculate distance away from second buoy
            x2 = x_list[ind_g2]
            y2 = -1*y_list[ind_g2]
            alpha2 = math.atan2(y2,x2) + math.pi/2
            if (abs(alpha2) > (math.pi)):
                alpha2 = (alpha2/abs(alpha2))*(abs(alpha2) - 2*math.pi)

            self.ned_alpha = alpha2 + self.yaw
            if (abs(self.ned_alpha) > (math.pi)):
                self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)

            xm2, ym2 = self.gate_to_body(0,0,alpha2,self.x_farther+x2,self.y_farther+y2)
            self.waypoints_second_buoy = [self.x_farther+x2,self.y_farther+y2,xm2,ym2]
            
            #Go to first waypoint from first buoy
            
            self.waypoints.guidance_law = 1
            self.waypoints.waypoint_list_length = 2
            self.waypoints.waypoint_list_x = [x1, xm1]
            self.waypoints.waypoint_list_y = [y1, ym1 - self.radius - self.long_sub]
            self.waypoints.waypoint_list_z = [0,0]  
            self.activation = False 
            self.desired(self.waypoints)
    
    def transport_second_bouys(self):
        '''
        @name: transport_second_bouys
        @brief: Transport to second buoy
        @param: --
        @return: --
        ''' 
        if(self.activation == True):
            x2 = self.waypoints_second_buoy[0]
            y2 = self.waypoints_second_buoy[1]
            xm2 = self.waypoints_second_buoy[2]
            ym2 = self.waypoints_second_buoy[3]
            self.waypoints.guidance_law = 1
            self.waypoints.waypoint_list_length = 2
            self.waypoints.waypoint_list_x = [x2, xm2]
            self.waypoints.waypoint_list_y = [y2, ym2 + self.radius + self.long_sub]
            self.waypoints.waypoint_list_z = [0,0]  
            self.activation = False 
            self.desired(self.waypoints)    

    def farther(self):
        '''
        @name: farther
        @brief: Returns a waypoint farther to the front of the vehicle in the NED
          reference frame to avoid perturbations.
        @param: --
        @return: --
        '''
        rospy.logwarn(self.y_farther)
        rospy.logwarn(self.x_farther)
        alpha2 = math.atan2(self.y_farther,self.x_farther) + math.pi/2
        if (abs(alpha2) > (math.pi)):
            alpha2 = (alpha2/abs(alpha2))*(abs(alpha2) - 2*math.pi)

        self.ned_alpha = alpha2 + self.yaw
        if (abs(self.ned_alpha) > (math.pi)):
            self.ned_alpha = (self.ned_alpha/abs(self.ned_alpha))*(abs(self.ned_alpha) - 2*math.pi)

        xm_farther, ym_farther = self.gate_to_body(0,0,alpha2,self.x_farther,self.y_farther)

        self.waypoints.guidance_law = 1
        self.waypoints.waypoint_list_length = 2
        self.waypoints.waypoint_list_x = [self.x_farther, xm_farther]
        self.waypoints.waypoint_list_y = [self.y_farther, ym_farther]
        self.waypoints.waypoint_list_z = [0, 0]  
        self.desired(self.waypoints)

    def gate_to_body(self, gate_x2, gate_y2, alpha, body_x1, body_y1):
        '''
        @name: gate_to_body
        @brief: Coordinate transformation between gate and body reference frames.
        @param: gate_x2: target x coordinate in gate reference frame
                gate_y2: target y coordinate in gate reference frame
                alpha: angle between gate and body reference frames
                body_x1: gate x coordinate in body reference frame
                body_y1: gate y coordinate in body reference frame
        @return: body_x2: target x coordinate in body reference frame
                 body_y2: target y coordinate in body reference frame
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = self.rotation_matrix(alpha)
        n = J.dot(p)
        body_x2 = n[0] + body_x1
        body_y2 = n[1] + body_y1
        return (body_x2, body_y2)

    def body_to_ned(self, x2, y2):
        '''
        @name: body_to_ned
        @brief: Coordinate transformation between body and NED reference frames.
        @param: x2: target x coordinate in body reference frame
                y2: target y coordinate in body reference frame
        @return: ned_x2: target x coordinate in ned reference frame
                 ned_y2: target y coordinate in ned reference frame
        '''
        p = np.array([x2, y2])
        J = self.rotation_matrix(self.yaw)
        n = J.dot(p)
        ned_x2 = n[0] + self.ned_x
        ned_y2 = n[1] + self.ned_y
        return (ned_x2, ned_y2)

    def gate_to_ned(self, gate_x2, gate_y2, alpha, ned_x1, ned_y1):
        '''
        @name: gate_to_ned
        @brief: Coordinate transformation between gate and NED reference frames.
        @param: gate_x2: target x coordinate in gate reference frame
                gate_y2: target y coordinate in gate reference frame
                alpha: angle between gate and ned reference frames
                body_x1: gate x coordinate in ned reference frame
                body_y1: gate y coordinate in ned reference frame
        @return: body_x2: target x coordinate in ned reference frame
                 body_y2: target y coordinate in ned reference frame
        '''
        p = np.array([[gate_x2],[gate_y2]])
        J = self.rotation_matrix(alpha)
        n = J.dot(p)
        ned_x2 = n[0] + ned_x1
        ned_y2 = n[1] + ned_y1
        return (ned_x2, ned_y2)

    def rotation_matrix(self, angle):
        '''
        @name: rotation_matrix
        @brief: Transformation matrix template.
        @param: angle: angle of rotation
        @return: J: transformation matrix
        '''
        J = np.array([[math.cos(angle), -1*math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]])
        return (J)

    def desired(self, path):
    	self.uuv_waypoints.publish(path)
        self.uuv_path.header.stamp = rospy.Time.now()
        self.uuv_path.header.frame_id = "world"
        del self.uuv_path.poses[:]
        for index in range(path.waypoint_list_length):
            pose = PoseStamped()
            pose.header.stamp       = rospy.Time.now()
            pose.header.frame_id    = "world"
            pose.pose.position.x    = path.waypoint_list_x[index]
            pose.pose.position.y    = path.waypoint_list_y[index]
            pose.pose.position.z    = path.waypoint_list_z[index]
            self.uuv_path.poses.append(pose)
        self.uuv_path_pub.publish(self.uuv_path)
    def generate_circle(self, _radius, _x_center, _y_center, _z_center):
        if(self.activation == True):    
            rospy.logwarn(_radius)
            rospy.logwarn(_x_center)
            rospy.logwarn(_y_center)
            rospy.logwarn(_z_center)
            waypointsCircle = GuidanceWaypoints()        
            angle = 0
            counter = 0 
            
            while (angle >= -self.PI):
                waypointsCircle.waypoint_list_x.append(_radius * math.sin(angle) + _x_center)
                waypointsCircle.waypoint_list_y.append(_radius * math.cos(angle) + _y_center)
                waypointsCircle.waypoint_list_z.append(_z_center)
                angle -= self.PI / 6
                counter+=1
        
            angle = self.PI

            while (angle >= 0):
                waypointsCircle.waypoint_list_x.append(_radius * math.sin(angle) + _x_center)
                waypointsCircle.waypoint_list_y.append(_radius * math.cos(angle) + _y_center)
                waypointsCircle.waypoint_list_z.append(_z_center)
                angle -= self.PI / 6
                counter += 1
            
            waypointsCircle.waypoint_list_length = counter
            waypointsCircle.guidance_law = 2
            self.activation = False
            self.desired(waypointsCircle)
    def turn(self, yaw):
        if(self.activation == True):
            self.x_farther += 1
            self.activation = False
            self.waypoints.guidance_law = 0
            self.waypoints.heading_setpoint = yaw
            self.waypoints.depth_setpoint = 0
            self.desired(self.waypoints)

def main():
    rospy.init_node("buoys", anonymous=False)
    rate = rospy.Rate(20)
    autoNav = AutoNav()
    last_detection = []
    selection = 0
    while not rospy.is_shutdown() and autoNav.activated:
        if selection  == 0:
            autoNav.activation = True
            autoNav.turn(-0.79)
            rospy.sleep(6.)
            selection = 1
        elif selection  == 1:
            autoNav.activation = True
            autoNav.turn(0.79)
            rospy.sleep(6.)
            selection = 2
        if selection == 2:
            autoNav.activation = True
            autoNav.farther()
            rospy.sleep(6.)
            selection = 0
            if(autoNav.x_farther == 4):
                selection = 3
        elif selection  == 3:
            autoNav.activation = True
            autoNav.calculate_two_bouys()
            rospy.sleep(6.)
            selection = 4
        elif selection  == 4:
            autoNav.activation = True
            autoNav.generate_circle(autoNav.radius, autoNav.waypoints_first_buoy[2], autoNav.waypoints_first_buoy[3], 0)
            rospy.sleep(6.)
            selection = 5
        elif selection  == 5:
            autoNav.activation = True
            autoNav.transport_second_bouys()
            rospy.sleep(6.)
            selection = 6
        elif selection  == 6:
            autoNav.activation = True
            autoNav.generate_circle(autoNav.radius, autoNav.waypoints_second_buoy[2], autoNav.waypoints_second_buoy[3], 0)
            rospy.sleep(6.)
            selection = 0
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

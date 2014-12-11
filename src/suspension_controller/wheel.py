# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#
# Based on code by Mattia Marenco:
# Copyright (c) 2013, Mattia Marenco <mattia.marenco@teamdiana.org>
# All rights reserved.

from numpy import average, pi
from utils import *

import rospy
from math import cos, acos, fabs

import time

import tf
from tf.transformations import euler_from_quaternion

from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as JointStateOut
from std_msgs.msg import Float64

from dynamixel_controllers.srv import SetTorque
from suspension_controller.srv import Freeze, SetMode, SetHeight, StopAll

from suspension_controller.constants import config
from circular_array import CircularArray
from rover_dimensions import *

class Wheel:
    def __init__(self, index, clockwise=True):
        self.torquef = CircularArray(config.TORQUE_SAMPLE_SIZE)
        self.torque = 0.0
        self.arm_pos_motor_radians = 0.0  # was self.pos_arm
        self.arm_error_motor_radians = 0.0  # was self.error_arm
        self.motor_temp = 0.0
        self.delta = 0.0
        self.arm_pos_rotary_sensor_radians = 0.0# was self.angoli_sosp
        self.phi = 0.0  # was self.fi
        self.pull_down_sts = False
        self.joint_state_out = JointStateOut()
        self.index = index
        self.clockwise = clockwise
        location_list = ['f_l', 'f_r', 'b_r', 'b_l']
        if not 1 <= self.index <= len(location_list):
            raise Exception("No location for index " + str(self.index)) 
        self.location = location_list[self.index-1]
        self.base_topic_name = "/motore_%d_controller" % self.index

    def read_wheel_data(self, msg):
        self.torquef.push_value(msg.load)
        avg = average(self.torquef.values)
        self.torque = avg
        self.arm_pos_motor_radians = msg.current_pos
        self.arm_error_motor_radians = msg.error
        self.motor_temp = int(msg.motor_temps[0])

    def init_publishers(self):
        self.command_wheel_pub = rospy.Publisher(self.base_topic_name + "/arm/command", 
                Float64)

    def init_subscribers(self):
        base_name = "/motore_%d_controller" % self.index
        self.wheel_status_sub = rospy.Subscriber(self.base_topic_name + "/arm/state" ,
                JointState, self.read_wheel_data)

    def init(self):
        self.init_publishers()
        self.init_subscribers()

    def on_exit(self):
        self.command_wheel_pub.unregister()
        self.wheel_status_sub.unregister()

    def stop(self):
        try:
            motor_motion = rospy.ServiceProxy('/motore_%d_controller/set_torque' % self.index, SetTorque)
            resp = motor_motion(False)
            rospy.loginfo("Suspended motor_%d" % self.index)
            return resp.response
        except rospy.ServiceException as e:
            rospy.logerr("Stop service call failed: %s" % e)
        return False

    #def delta_follower(self, limit):
        #if self.error > limit or self.error < -limit:
            #self.delta = self.error

    #def read_suspension_angle(self, msg):
        #self.suspension_angle = getattr(msg, "sosp%d" % self.index)

    def get_chassis_height(self):
        return self.current_chassis_height

    def read_tf_data(self, range_front, range_rear,
            requested_chassis_height, tf_listener, tf_broadcaster):
        rover_dim = get_rover_dimensions()
        base_link_tf_name = '/base_link'
        chassis_tf_name = '/rover_amalia_chassis'
        wheel_tf_name = '/rover_amalia_leg_wheel_%s' % self.location
        hub_tf_name = '/rover_amalia_leg_bar_%s' % self.location

        try:
            (chassis_wheel_trans, _) = get_transform(tf_listener, chassis_tf_name, 
                    wheel_tf_name)
            suspension_hub_height = -chassis_wheel_trans[2] + rover_dim['hub_chassis_center_z']
            #rospy.loginfo("suspension_hub_height of wheel %i is %f", 
                    #self.index, suspension_hub_height)
        except Exception as e: 
            rospy.logerr('no_info from %s to %s: %s', chassis_tf_name, wheel_tf_name, e.message)
            return False

        try:
            (base_link_hub_trans, _) = get_transform(tf_listener, 
                    base_link_tf_name, hub_tf_name)
            (base_link_wheel_trans, _) = get_transform(tf_listener, 
                    base_link_tf_name, wheel_tf_name)
            hub_proj_z = -base_link_hub_trans[2]  
            wheel_proj_z = -base_link_wheel_trans[2]  
            #rospy.loginfo("wheel %i length %f - hub_proj_z: %f - wheel_proj_z %f" , self.index, 
                   #rover_dim['wheel_center_hub_dist'] , hub_proj_z, wheel_proj_z)
            #rospy.loginfo("wheel %i wheel %f %f %f" , self.index, 
                   #base_link_wheel_trans[0], base_link_wheel_trans[1], base_link_wheel_trans[2])
            self.wheel_height = rover_dim['wheel_center_hub_dist'] - wheel_proj_z + hub_proj_z

        except Exception as e:
            rospy.logerr('unable to know height of wheel %i: %s', self.index, e.message)
            return False

        try:
            (_, chassis_rot) = get_transform(tf_listener, base_link_tf_name, chassis_tf_name)
            #rospy.logdebug("Chassis position: %f %f %f", trans[0], trans[1], trans[2])

            chassis_angles = euler_from_quaternion(chassis_rot)

            #rospy.logdebug("Chassis orientation by chassis_angles: %f %f %f", 
                    #chassis_angles[0], chassis_angles[1], chassis_angles[2])
            proj_v = cos(fabs(chassis_angles[1])) * cos(fabs(chassis_angles[0]))
            range_chassis_center_z = get_rover_dimensions()['range_chassis_center_z']
            proj_range_front = range_chassis_center_z + range_front * proj_v
            proj_range_rear = range_chassis_center_z + range_rear * proj_v
            #self.current_chassis_height = min(proj_range_front, proj_range_rear, suspension_hub_height) 
            self.current_chassis_height = min(proj_range_front, proj_range_rear) 
            chassis_height_0deg = 0.314
            self.wheel_height -= chassis_height_0deg - self.current_chassis_height

            #rospy.loginfo("Computed height from range finder module: %f", self.current_chassis_height)
        except Exception as e: 
            rospy.logerr("no_info base_link to chassis: %s", e.message)
            return False

        self.chassis_angles = chassis_angles
        requested_current_chassis_height_delta = requested_chassis_height - self.current_chassis_height
        rospy.logdebug("requested height - current height: %f", requested_current_chassis_height_delta)
        return True


    def compute_phi(self, req_chassis_height):
        rover_dim = get_rover_dimensions()
        req_wheel_proj_z = (req_chassis_height - rover_dim['hub_chassis_center_z'] 
                                               - rover_dim['wheel_radius']
                                               - self.wheel_height)
        #rospy.loginfo("req_wheel_proj_z phi for wheel %i is %f" , self.index, req_wheel_proj_z)
        req_wheel_proj_z_arm_length_delta = req_wheel_proj_z - rover_dim['wheel_center_hub_dist']
        if(req_wheel_proj_z_arm_length_delta > 0):
            if(req_wheel_proj_z_arm_length_delta > 0.05):
                rospy.logerr("ERROR: req_wheel_proj_z > hub_chassis_center_z")
                return False
            else:
                req_wheel_proj_z = rover_dim['wheel_center_hub_dist']
        rat = req_wheel_proj_z  / rover_dim['wheel_center_hub_dist']
        rospy.loginfo( "WHEEL %i - wheel_height %f - req_wheel_proj_z %f - req_wheel_proj_delta %f - rat %f", 
                self.index, self.wheel_height, req_wheel_proj_z, req_wheel_proj_z_arm_length_delta, rat)
        #rospy.loginfo("rat for wheel %i is %f" , self.index, rat)
        if fabs(rat) < 1:
            self.phi = acos(rat)
        else:
            self.phi = 0
        if not self.clockwise:
            self.phi *= -1
        #rospy.loginfo("Calculated phi for wheel %i is %f" , self.index, self.phi)
        return True

        #try:
            #self.phi = arccos((self.posa_chassis_virtuale[2] - 0.011 - self.wheel_height) / 0.20)
        #except ValueError:
            #rospy.logwarn("out_of_range on wheel %i", self.index)
            # TODO: What is the if statement trying to do?
            #if ((self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote) / 0.20) > 1.:
                #self.phi = config.MIN_WHEEL_ANGLE
            #elif ((self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote) / 0.20) < 0.:
                #self.phi = config.MAX_WHEEL_ANGLE

        #joint_state_out = JointStateOut()

        #joint_state_out.name = [ "hub_%s_virtuale" % self.location ]
        #joint_state_out.position = [ self.phi + self.delta ]
        #joint_state_out.velocity = []
        #joint_state_out.effort = []
        #joint_state_out.header.stamp = rospy.Time.now()
        #joint_state_out_pub.publish(joint_state_out)

    def publish_phi(self):
        if (self.pull_down_sts == False):
            #phi = self.phi + self.delta
            phi = self.phi
            phi = clamp(config.MIN_PHI_ANGLE, config.MAX_PHI_ANGLE, self.phi)

            # TODO: important! reenable this check 
            #if self.torque > config.TORQUE_MAX_LOAD:
                #rospy.logwarn("Limite coppia su %i!", self.index)
                #phi = phi + 0.1
            #rospy.loginfo("Outputting phi for wheel %i is %f" , self.index, phi)
            self.command_wheel_pub.publish(phi)

            #if self.torque > config.TORQUE_MAX_LOAD:
                #time.sleep(0.5)


    #def pull_down(self):
        #if self.torque < 0.1:
            #self.pull_down_sts = True
            #phi = self.position - 0.5
            #if phi < config.MIN_PHI_ANGLE:
                #phi = config.MIN_PHI_ANGLE
            #elif phi > config.MAX_PHI_ANGLE:
                ## XXX: Shouldn't this be config.MAX_PHI_ANGLE?
                #phi = 1.00

            #rospy.logdebug("Coppia motore %i %f -> comando posizione %f", self.index, self.torque, phi)
            #self.command_wheel_pub.publish(phi)

            #self.joint_state_out.name = [ "hub_%s_virtuale" % self.location ]
            #self.joint_state_out.position = [ phi ]
            #self.joint_state_out.velocity = []
            #self.joint_state_out.effort = []
            #self.joint_state_out.header.stamp = rospy.Time.now()
            #self.joint_state_out_pub.publish(self.joint_state_out)

        #else:
            #self.pull_down_sts = False

        #time.sleep(0.1)

#!/usr/bin/env python

# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#
# Based on code by Mattia Marenco:
# Copyright (c) 2013, Mattia Marenco <mattia.marenco@teamdiana.org>
# All rights reserved.

import rospy

from numpy import fabs

from suspension_controller.srv import Freeze, SetMode, SetHeight, StopAll

import time

import tf
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Range
#from adc.msg import sosp_Adc
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from suspension_controller.msg import Status

from suspension_controller.suspension_modes import BaseMode, Observer, Follower, Simulation, WithAntilift, WithAntiliftAndFollower, WithFollower

from suspension_controller.wheel import Wheel

from suspension_controller.constants import config

from topics_name import get_topics_names

class SuspensionController:
    def __init__(self):
        rospy.init_node('suspension_controller', anonymous=True, log_level=rospy.INFO)
        rospy.on_shutdown(self.on_exit)

        self.hz = 4
        self.rate = rospy.Rate(self.hz)

        self.chassis_angles = [ 0.0, 0.0, 0.0 ]  # was self.angoli_chassis
        self.range_front_value = self.range_rear_value = 0.254
        
        self.status_asm = Status()

        self.requested_chassis_height = 0.314

        self.chassis_height_0deg = self.current_chassis_height = self.requested_chassis_height
        self.recovery_height = False
        self.freeze = False

        self.wheels = [ Wheel(1, False), Wheel(2, False), Wheel(3), Wheel(4) ]
        # TODO: Figure out a better way to wait for valid messages from the ADC
        time.sleep(config.INIT_ADC_WAIT_PERIOD)

        self.init_publishers()

        self.available_modes = [Simulation, Follower, Observer, 
                WithAntilift, WithFollower, WithAntiliftAndFollower] 
        self.controller_mode = Observer(self)
        self.last_update_time = rospy.Time.now()

    def run(self):
        rospy.loginfo("run()")
        while not rospy.is_shutdown():
            self.get_tf()
            if self.is_update_required():
                self.last_update_time = rospy.Time.now()
                rospy.loginfo("update is required: doing new controller cycle")
                self.controller_mode.do_controller_cycle()
            else:
                rospy.loginfo("update is not required...")
            self.rate.sleep()


    def init_publishers(self):
        '''
        This function starts all ROS publishers, services, subscribers, listeners and broadcasters. This function needs to be called early in the initialization process.
        '''
        for wheel in self.wheels:
            wheel.init()

        topic_names = get_topics_names()
        #self.joint_state_out_pub = rospy.Publisher('/joint_states', JointState)
        #self.status_asm_pub = rospy.Publisher('/status_asm', Status)

        self.range_front_sub = rospy.Subscriber(topic_names['range_front'], 
                Range, self.process_range_front)
        self.range_rear_sub = rospy.Subscriber(topic_names['range_rear'], 
                Range, self.process_range_rear)
        #self.sus_sub = rospy.Subscriber('/ADC/suspension', sosp_Adc, self.read_suspension_angles)

        self.service_height = rospy.Service('suspension_controller/set_height', SetHeight, self.handle_set_height)
        self.service_mode = rospy.Service('suspension_controller/set_mode', SetMode, self.handle_set_mode)
        self.service_stop = rospy.Service('suspension_controller/stop_all', StopAll, self.handle_stop_all)
        #self.service_freeze = rospy.Service('suspension_controller/freeze', Freeze, self.handle_freeze)

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()


    def on_exit(self):
        for wheel in self.wheels:
            wheel.on_exit()

        #self.joint_state_out_pub.unregister()
        #self.status_asm_pub.unregister()

        self.range_front_sub.unregister()
        self.range_rear_sub.unregister()
        #self.sus_sub.unregister()

    def handle_set_height(self, req):
        if req.height > self.chassis_height_0deg:
            rospy.logerr("Requested height %f > maximum chassis height", 
                    req.height)
            return False
        self.requested_chassis_height = req.height 
        rospy.loginfo("Height set at %f.", self.requested_chassis_height)
        return True

    def handle_set_mode(self, req):
        if not 0 <= req.mode < len(self.available_modes):
            raise Exception("Mode " + str(req.mode) + "not recognized!")
        self.controller_mode = self.available_modes[req.mode](self)
        rospy.loginfo("Operation mode set to %s.", self.controller_mode.name)
        return True


    def handle_stop_all(self, req):
        '''
        This handler tries to stop the motion of all the wheels. It handles requests for the StopAll service.
        It returns True if and only if it succeeds in stopping all wheels.
        ''' 
        responses = []
        ok = True
        for wheel in self.wheels:
            resp = wheel.stop()
            if not resp:
                rospy.logerr("Unable to stop wheel " + wheel.index)
                ok = False
        if ok:
            rospy.loginfo("All wheels stopped")
            return True
        else: 
            return False

    # TODO: figure out the logic
    #def handle_freeze(self, req):
        #self.freeze = req.freeze

        #if self.freeze:
            #rospy.loginfo("System freeze requested")
            #self.get_tf()
            #self.calculate_phi()
##             self.delta = ([0.0] * 4)
            #self.output_phi()
            #while min(self.torque) < 0.1 :
                #self.pull_down()
        #return self.freeze


    #def test_wheels(self, init_pos_high=True):
        #rospy.loginfo("Testing wheels with initial position high? %b", init_pos_high)

        #for wheel in self.wheels:
            #wheel.test_wheel(init_pos_high)

    #def diagnostics_processor(self):
        #self.rate.sleep()

    def process_range_front(self, msg):
        self.range_front_value = msg.range

    def process_range_rear(self, msg):
        self.range_rear_value = msg.range

    def read_suspension_angles(self, msg):
        for wheel in self.wheels:
            wheel.read_suspension_angle(msg)

    def get_chassis_angles(self):
        try:
            (_, chassis_rot) = self.tf_listener.lookupTransform('base_link', 
                    'rover_amalia_chassis', rospy.Time(0))
            return euler_from_quaternion(chassis_rot)
        except Exception as e: 
            err_msg = "no_info base_link to chassis: %s" % e.message
            rospy.logerr(err_msg)
            raise Exception(err_msg)

    def get_tf(self):
        #rospy.loginfo("get TF")
        #rospy.loginfo("RANGE VALUES: front %f   rear %f " , 
                #self.range_front_value,
                #self.range_rear_value)
        for wheel in self.wheels:
            ok = wheel.read_tf_data(self.range_front_value, self.range_rear_value, 
                    self.requested_chassis_height,  self.tf_listener, self.tf_broadcaster)
            if not ok:
                err_msg = "Unable to read tf data"
                rospy.logerr(err_msg)
                raise Exception(err_msg)
        chassis_heights_for_wheels = [w.get_chassis_height() for w in self.wheels]
        self.current_chassis_height = min(chassis_heights_for_wheels)
        #rospy.loginfo("Current chassis height is %f" % self.current_chassis_height )
        self.chassis_angles = self.get_chassis_angles()

    def is_out_of_allowed_limits(self):
        limit = 0.05
        height_limit = 0.05
        roll = self.chassis_angles[0]
        pitch = self.chassis_angles[1]
        requested_current_height_delta = self.requested_chassis_height - self.current_chassis_height
        return (fabs(pitch) > limit or 
                fabs(roll) > limit or 
                fabs(requested_current_height_delta) > height_limit)

    def is_update_required(self):
        if self.is_out_of_allowed_limits():
            rospy.loginfo("is out of limits. update required")
            return True
        if (rospy.Time.now() - self.last_update_time).secs > 1: 
            rospy.loginfo("too much time passed. update required")
            return True
        return False

    def update_wheels_phi(self):
        rospy.loginfo("updating wheels phi")
        for wheel in self.wheels:
            wheel.compute_phi(self.requested_chassis_height)
            wheel.publish_phi()

    def pull_down(self):
        for wheel in self.wheels:
            wheel.pull_down()

    def follower(self):
        limit = 0.03
        for wheel in self.wheels:
            wheel.delta_follower(limit)


    # TODO: inserire is_moving ed abilitare il reset del filtro
    # TODO: inserire temperature e lettura delle stesse
    #def update_status(self):
        #for wheel in self.wheels:
            #wheel.update_status(self.status_asm)

        #self.status_asm.command_height = self.requested_chassis_height - 0.05

        #deltaH_inertial = []
        #for wheel in self.wheels:
            #deltaH_inertial.append(wheel.deltaH_inertial)

        #self.status_asm.inertial_height = max(deltaH_inertial)

        #self.status_asm.mode = self.mode
        #self.status_asm.header.stamp = rospy.Time.now()
        #self.status_asm_pub.publish(self.status_asm)

if __name__ == '__main__':
    try:
        controller = SuspensionController()
        controller.run()
    except rospy.ROSInterruptException: pass

#!/usr/bin/env python3

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

from sensor_msgs.msg import Range
from adc.msg import sosp_Adc
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from suspension_controller.msg import Status

from suspension_controller.suspension_modes import BaseMode, Observer, Follower, Simulation, SuspensionMode, WithAntilift, WithAntiliftAndFollower, WithFollower

from suspension_controller.wheel import Wheel

from suspension_controller.constants.config import INIT_ADC_WAIT_PERIOD

class SuspensionController:
    def __init__(self):
        rospy.init_node('suspension_controller', anonymous=True, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.unpublish)

        # This is the same as dynamixel
        self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1)

        # This should be the same for all engines, so we only get the speed of the first wheel
        self.speed = rospy.get_param('/motore_1_controller/joint_speed', 0.5)
        self.hz = 40
        self.rate = rospy.Rate(self.hz)

        self.chassis_angles = [ 0.0, 0.0, 0.0 ]  # was self.angoli_chassis
        self.range_front = 0.4
        self.range_post = 0.4

        self.status_asm = Status()

        self.req_height = 0.25
        self.req_height_temp = 0.25
        self.recovery_height = False
        self.mode = 0
        self.freeze = False

        self.wheels = [ Wheel(1), Wheel(2), Wheel(3), Wheel(4) ]

        rospy.loginfo("INIT")

        # TODO: Figure out a better way to wait for valid messages from the ADC
        time.sleep(INIT_ADC_WAIT_PERIOD)

        self.publish()
        self.start()


    def publish(self):
        '''
        This function starts all ROS publishers, services, subscribers, listeners and broadcasters. This function needs to be called early in the initialization process.
        '''
        for wheel in self.wheels:
            wheel.publish()

        self.joint_state_out_pub = rospy.Publisher('/joint_states', JointState)
        self.status_asm_pub = rospy.Publisher('/status_asm', Status)

        self.range_front_sub = rospy.Subscriber('/ADC/range_front_down', Range, self.process_range_front)
        self.range_post_sub = rospy.Subscriber('/ADC/range_post_down', Range, self.process_range_post)
        self.sus_sub = rospy.Subscriber('/ADC/suspension', sosp_Adc, self.read_suspension_angles)

        self.service_height = rospy.Service('suspension_controller/set_height', SetHeight, self.handle_set_height)
        self.service_mode = rospy.Service('suspension_controller/set_mode', SetMode, self.handle_set_mode)
        self.service_stop = rospy.Service('suspension_controller/stop_all', StopAll, self.handle_stopAll)
        self.service_freeze = rospy.Service('suspension_controller/freeze', Freeze, self.handle_freeze)

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.running = True


    # TODO: check if I need to stop the tf listeners and broadcasters
    def unpublish(self):
        '''
        This function stops all ROS publishers, services, subscribers, listeners and broadcasters. This function needs to be called early in the initialization process.when the node is stopped.
        '''
        for wheel in self.wheels:
            wheel.unpublish()

        self.joint_state_out_pub.unregister()
        self.status_asm_pub.unregister()

        self.range_front_sub.unregister()
        self.range_post_sub.unregister()
        self.sus_sub.unregister()

        self.service_height.unregister()
        self.service_mode.unregister()
        self.service_stop.unregister()
        self.service_freeze.unregister()

        self.running = False


    # TODO: what is the 0.05?
    # TODO: do I have to return anything?
    def handle_set_height(self, req):
        '''
        This is a simple handler to set the height of the rover.
        It always returns True.
        '''
        self.req_height = req.height + 0.05
        rospy.loginfo("Height set at %f.", self.req_height)
#         return True


    def handle_set_mode(self, req):
        '''
        This is a simple method that converts the value received from the service to python.
        '''
        self.mode = req.mode
        rospy.loginfo("Operation mode set to %s.", self.mode)


    def handle_stop_all(self, req):
        '''
        This handler tries to stop the motion of all the wheels. It handles requests for the StopAll service.
        It returns True if and only if it succeeds in stopping all wheels.
        '''
        responses = []
        res = True
        for wheel in self.wheels:
            resp = wheel.stop()
            # if we have failed to stop one of them, save it to avoid bitwise AND, but still try to stop the others
            if resp is None:
                res = False
            responses.append(resp)

        # TODO: check logic: if res is true then all responses are true, else then always false? Why did it use bitwise AND?
        return res
#         if res == True:
#             return True # responses[0] & responses[1] & responses[2] & responses[3]
#         else:
#             return False


    # TODO: figure out the logic
    def handle_freeze(self, req):
        self.freeze = req.freeze

        if self.freeze:
            rospy.loginfo("System freeze requested")
            self.get_tf()
            self.calculate_phi()
#             self.delta = ([0.0] * 4)
            self.output_phi()
            while min(self.torque) < 0.1 :
                self.pull_down()
        return self.freeze


    def test_wheels(self, init_pos_high=True):
        rospy.loginfo("Testing wheels with initial position high? %b", init_pos_high)

        for wheel in self.wheels:
            wheel.test_wheel(init_pos_high)


    def diagnostics_processor(self):
        self.rate.sleep()


    def process_range_front(self, msg):
        self.range_front = msg.range


    def process_range_post(self, msg):
        self.range_post = msg.range


    def read_suspension_angles(self, msg):
        for wheel in self.wheels:
            wheel.read_suspension_angle(msg)


    # TODO: check that this does what it's supposed to do
    def get_tf(self):
        rospy.loginfo("get TF")
        for wheel in self.wheels:
            wheel.compute_transfer_function(self.range_front, self.range_post, self.req_height, self.joint_state_out_pub, self.tf_listener, self.tf_broadcaster)
            self.deltaH_chassis_virtuale = wheel.deltaH_chassis_virtuale
            self.chassis_angles = wheel.chassis_angles


    def calculate_phi(self):
        roll = self.chassis_angles[0]
        pitch = self.chassis_angles[1]
        limit = 0.05

        for wheel in self.wheels:
            if fabs(pitch) > limit or fabs(roll) > limit or fabs(self.deltaH_chassis_virtuale) > limit:
                wheel.publish_phi(self.joint_state_out_pub)


    def output_phi(self):
        for wheel in self.wheels:
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
    def update_status(self):
        for wheel in self.wheels:
            wheel.update_status(self.status_asm)

        self.status_asm.command_height = self.req_height - 0.05

        deltaH_inertial = []
        for wheel in self.wheels:
            deltaH_inertial.append(wheel.deltaH_inertial)

        self.status_asm.inertial_height = max(deltaH_inertial)

        self.status_asm.mode = self.mode
        self.status_asm.header.stamp = rospy.Time.now()
        self.status_asm_pub.publish(self.status_asm)


    def start(self):
        mode = SuspensionMode(BaseMode)
        while not rospy.is_shutdown():
            if not self.freeze:
                if self.mode == 0:
                    mode.set(Simulation(self))
                    mode.run()
                elif self.mode == 1:
                    mode.set(Follower(self))
                    mode.run()
                elif self.mode == 2:
                    mode.set(Observer(self))
                    mode.run()
                elif self.mode == 3:
                    mode.set(WithAntilift(self))
                    mode.run()
                elif self.mode == 4:
                    mode.set(WithFollower(self))
                    mode.run()
                elif self.mode == 5:
                    mode.set(WithAntiliftAndFollower(self))
                    mode.run()
                else:
                    raise Exception("Mode not recognized!")

            self.update_status()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        SuspensionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass
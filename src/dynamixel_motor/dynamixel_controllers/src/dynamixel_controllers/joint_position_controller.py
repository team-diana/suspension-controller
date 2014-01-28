# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division


__author__ = 'Mattia Marenco, Antons Rebguns'
__copyright__ = 'Copyright (c) 2013 Mattia Marenco'

__license__ = 'BSD'
__maintainer__ = 'Mattia Marenco'
__email__ = 'mattia.marenco@teamdiana.org'

import rospy

import time

import math

from dynamixel_driver.dynamixel_const import *
from dynamixel_controllers.joint_controller import JointController

from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as JointStateOut
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped

old_speed = 0.0

class JointStateMessage():
     def __init__(self, name, position, velocity, effort):
         self.name = name
         self.position = position
         self.velocity = velocity
         self.effort = effort

class JointPositionController(JointController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointController.__init__(self, dxl_io, controller_namespace, port_namespace)
        
        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        self.initial_position_raw = rospy.get_param(self.controller_namespace + '/motor/init')
        self.min_angle_raw = rospy.get_param(self.controller_namespace + '/motor/min')
        self.max_angle_raw = rospy.get_param(self.controller_namespace + '/motor/max')
        
        self.flipped = self.min_angle_raw > self.max_angle_raw
        self.last_commanded_torque = 0.0
        
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])
        self.arm_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])
        self.joint_state_out = JointStateOut()
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        self.wrench_state = WrenchStamped()
        self.wrench_state.header.frame_id = ""
        self.wrench_state.wrench.force.x = 0
        self.wrench_state.wrench.force.y = 0
        self.wrench_state.wrench.force.z = 0
        self.wrench_state.wrench.torque.x = 0
        self.wrench_state.wrench.torque.y = 0
        self.wrench_state.wrench.torque.z = 0

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        self.RADIANS_PER_ENCODER_TICK = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.ENCODER_TICKS_PER_RADIAN = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))
        
        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            
        self.ENCODER_RESOLUTION = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = rospy.get_param('dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.motor_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK
        
        #TODO inserire qui i valori ottimati ricavati per la slope e valore massimo per torque limit se necessario
        
        if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
        if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)
        
        self.threshold_degree = 20 #start value soglia = 20°
        
        self.dxl_io.set_d_gain(self.motor_id, 0)  #40
        self.dxl_io.set_i_gain(self.motor_id, 2)  #20
        self.dxl_io.set_p_gain(self.motor_id, 5) #100
        
        self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)
        
        if self.joint_max_speed < self.MIN_VELOCITY: self.joint_max_speed = self.MIN_VELOCITY
        elif self.joint_max_speed > self.MAX_VELOCITY: self.joint_max_speed = self.MAX_VELOCITY
        
        if self.joint_speed < self.MIN_VELOCITY: self.joint_speed = self.MIN_VELOCITY
        elif self.joint_speed > self.joint_max_speed: self.joint_speed = self.joint_max_speed
        
        self.old_speed = self.joint_speed         
        self.set_speed(self.joint_speed)
        
        return True

    def pos_rad_to_raw(self, pos_rad):
        if pos_rad < self.min_angle: pos_rad = self.min_angle
        elif pos_rad > self.max_angle: pos_rad = self.max_angle
        return self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def spd_rad_to_raw(self, spd_rad):
        if spd_rad < self.MIN_VELOCITY: spd_rad = self.MIN_VELOCITY
        elif spd_rad > self.joint_max_speed: spd_rad = self.joint_max_speed
        # velocity of 0 means maximum, make sure that doesn't happen
        return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))

    def spd_rad_to_raw_tor(self, spd_rad):
        if spd_rad < -self.joint_max_speed: spd_rad = -self.joint_max_speed
        elif spd_rad > self.joint_max_speed: spd_rad = self.joint_max_speed
        self.last_commanded_torque = spd_rad
        return int(round(spd_rad / self.VELOCITY_PER_TICK))

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id, torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv])

    def set_speed(self, speed):
        mcv = (self.motor_id, self.spd_rad_to_raw(speed))
        self.dxl_io.set_multi_speed([mcv])

    def set_speed_tor(self, speed):
        mcv = (self.motor_id, self.spd_rad_to_raw_tor(speed))
        self.dxl_io.set_multi_speed([mcv])

    def set_compliance_slope(self, slope):
        if slope < DXL_MIN_COMPLIANCE_SLOPE: slope = DXL_MIN_COMPLIANCE_SLOPE
        elif slope > DXL_MAX_COMPLIANCE_SLOPE: slope = DXL_MAX_COMPLIANCE_SLOPE
        mcv = (self.motor_id, slope, slope)
        self.dxl_io.set_multi_compliance_slopes([mcv])

    def set_compliance_margin(self, margin):
        if margin < DXL_MIN_COMPLIANCE_MARGIN: margin = DXL_MIN_COMPLIANCE_MARGIN
        elif margin > DXL_MAX_COMPLIANCE_MARGIN: margin = DXL_MAX_COMPLIANCE_MARGIN
        else: margin = int(margin)
        mcv = (self.motor_id, margin, margin)
        self.dxl_io.set_multi_compliance_margins([mcv])

    def set_compliance_punch(self, punch):
        if punch < DXL_MIN_PUNCH: punch = DXL_MIN_PUNCH
        elif punch > DXL_MAX_PUNCH: punch = DXL_MAX_PUNCH
        else: punch = int(punch)
        mcv = (self.motor_id, punch)
        self.dxl_io.set_multi_punch([mcv])

    def set_torque_limit(self, max_torque):
        if max_torque > 1: max_torque = 1.0         # use all torque motor can provide
        elif max_torque < 0: max_torque = 0.0       # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id, raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv])
        
    def set_torque(self, torque):
        self.dxl_io.set_torque_enabled(self.motor_id, torque)
        
    def set_threshold(self, threshold):
        self.threshold_degree = threshold

    def process_motor_states(self, state_list):
        self.state_list = state_list;
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = state.error * self.RADIANS_PER_ENCODER_TICK
                self.joint_state.velocity = (state.speed / DXL_MAX_SPEED_TICK) * self.MAX_VELOCITY
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)
                
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state_out.name = []
                self.joint_state_out.position = []
                self.joint_state_out.velocity = []
                self.joint_state_out.effort = []
                if self.motor_id == 1 or self.motor_id == 4:  
                    self.arm_state.motor_temps = [state.temperature]
                    
                    self.arm_state.error = state.error * self.RADIANS_PER_ENCODER_TICK / 6.3
                    self.arm_state.velocity = (state.speed / DXL_MAX_SPEED_TICK) * self.MAX_VELOCITY / 6.3
                    self.joint_state_out.velocity.append(self.arm_state.velocity)
                    self.arm_state.load = state.load * 6.3
                    self.joint_state_out.effort.append(self.arm_state.load)
                    self.arm_state.is_moving = state.moving
                    self.arm_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                    self.wrench_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                    self.joint_state_out.header.stamp = rospy.Time.now()
                    if self.motor_id == 1:
                         zero = (2.0*3.1416-4.6571)/6.3
                         self.arm_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK) / 6.3 + zero
                         self.arm_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK) / 6.3 + zero
                         self.arm_state.name = "hub_f_l"
                         self.joint_state_out.name.append(self.arm_state.name)
                         self.wrench_state.header.frame_id  = "leg_f_l"
                         self.wrench_state.wrench.torque.y = state.load * 6.3
                         self.wrench_state.wrench.force.x = state.load * 6.3 * 0.20
                    if self.motor_id == 4:
                         zero = (2.0*3.1416-3.9178)/6.3
                         self.arm_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK) / 6.3 + zero
                         self.arm_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK) / 6.3 + zero
                         self.arm_state.name = "hub_p_r"
                         self.joint_state_out.name.append(self.arm_state.name)
                         self.wrench_state.header.frame_id = "leg_p_r"
                         self.wrench_state.wrench.torque.y = - state.load * 6.3
                         self.wrench_state.wrench.force.x = - state.load * 6.3 * 0.20
                else:    
                    self.arm_state.motor_temps = [state.temperature]
                    
                    self.arm_state.error = state.error * self.RADIANS_PER_ENCODER_TICK / 6.3
                    self.arm_state.velocity = - (state.speed / DXL_MAX_SPEED_TICK) * self.MAX_VELOCITY / 6.3
                    self.joint_state_out.velocity.append(self.arm_state.velocity)
                    self.arm_state.load = - state.load * 6.3
                    self.joint_state_out.effort.append(self.arm_state.load)
                    self.arm_state.is_moving = state.moving
                    self.arm_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                    self.wrench_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                    self.joint_state_out.header.stamp = rospy.Time.now()
                    if self.motor_id == 2:
                         zero = -(-2.0831)/6.3
                         self.arm_state.goal_pos = (6.2832 + self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)) / 6.3 + zero
                         self.arm_state.current_pos = (6.2832 +  self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)) / 6.3 + zero
                         self.arm_state.name = "hub_p_l"
                         self.joint_state_out.name.append(self.arm_state.name)
                         self.wrench_state.header.frame_id = "leg_p_l"
                         self.wrench_state.wrench.torque.y = - state.load * 6.3
                         self.wrench_state.wrench.force.x = state.load * 6.3 * 0.20
                    if self.motor_id == 3:
                         zero = -(-2.2733)/6.3
                         self.arm_state.goal_pos = (6.2832 +  self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)) / 6.3 + zero
                         self.arm_state.current_pos = (6.2832 +  self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)) / 6.3 + zero
                         self.arm_state.name = "hub_f_r"
                         self.joint_state_out.name.append(self.arm_state.name)
                         self.wrench_state.header.frame_id = "leg_f_r"
                         self.wrench_state.wrench.torque.y = state.load * 6.3
                         self.wrench_state.wrench.force.x = - state.load * 6.3 * 0.20
                
                if (self.armOK): #sostituisco posizione con sensori
                    self.arm_state.error = self.arm_state.current_pos - self.arm[self.motor_id-1]
                    self.arm_state.current_pos = self.arm[self.motor_id-1]
                    self.joint_state_out.position.append(self.arm_state.current_pos)
                self.arm_state_pub.publish(self.arm_state)  
                self.joint_state_out_pub.publish(self.joint_state_out)
                self.wrench_state_pub.publish(self.wrench_state)
                
                #TODO aggiungere 4 arrow allineate con la direzione meccanica delle sospensioni teoriche come qui http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
                #TODO aggiungere controllo di plausibilit\E0 angoli sospensioni in base a input accelerometri, se ko sostituire i dati degli accelerometri alla posizione e notificare in diag?
                        

    def process_command(self, msg):
        angle = msg.data
        mcv = (self.motor_id, self.pos_rad_to_raw(angle))
        self.dxl_io.set_multi_position([mcv])

    def process_arm_command(self, msg):
        if self.motor_id == 1:
             zero = (2.0*3.1416-4.6571)/6.3
             angle = (msg.data - zero) * 6.3  
        if self.motor_id == 2:
             zero = -(-2.0831)/6.3 + (2.0*3.1416)/6.3
             angle = (msg.data - zero) * 6.3 
        if self.motor_id == 3:
             zero = -(-2.2733)/6.3 + (2.0*3.1416)/6.3
             angle = (msg.data - zero) * 6.3 
        if self.motor_id == 4:
             zero = (2.0*3.1416-3.9178)/6.3
             angle = (msg.data - zero) * 6.3    
        
        current_pos = self.joint_state.current_pos #self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
        threshold = self.threshold_degree*(2*math.pi)/360  #conversione da gradi a radianti della soglia
        modulated_speed = self.old_speed*(math.fabs(angle-current_pos)*threshold)   #modula da 0 a soglia e poi satura
        if modulated_speed < self.MIN_VELOCITY: modulated_speed = self.MIN_VELOCITY
        elif modulated_speed > self.old_speed: modulated_speed = self.old_speed
        self.set_speed(modulated_speed)
        
        mcv = (self.motor_id, self.pos_rad_to_raw(angle))
        self.dxl_io.set_multi_position([mcv])

    def process_step_command(self, msg):
        self.old_speed = self.joint_speed   
        self.dxl_io.set_angle_limit_ccw(self.motor_id, 0)
        self.set_speed_tor(msg.data)
        print("MODALITa VELOCITA")
        time.sleep(2);
        self.set_speed_tor(0)
        time.sleep(2);
        self.dxl_io.set_angle_limit_ccw(self.motor_id, 4095)
        
        self.set_speed(self.old_speed)
#        angle = 3.0
#        mcv = (self.motor_id, self.pos_rad_to_raw(angle))
#        self.dxl_io.set_multi_position([mcv])

    def process_arm_states(self, msg):
        self.armOK = True
        self.arm[0] = msg.sosp1
        self.arm[1] = msg.sosp2
        self.arm[2] = msg.sosp3
        self.arm[3] = msg.sosp4
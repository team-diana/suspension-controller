#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Mattia Marenco
# <mattia.marenco@teamdiana.org>
# All rights reserved.

__author__ = 'Mattia Marenco'
__copyright__ = 'Copyright (c) 2013 Mattia Marenco'

__license__ = 'BSD'
__maintainer__ = 'Mattia Marenco'
__email__ = 'mattia.marenco@teamdiana.org'


from threading import Thread

from array import *

import rospy
import roslib

from suspension_controller.srv import set_mode
from suspension_controller.srv import set_height
from suspension_controller.srv import stopAll
from dynamixel_controllers.srv import SetTorque
from adc.srv import movingService

import sys

import time
from numpy import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from adc.msg import sosp_Adc
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as JointStateOut

import tf
from tf.transformations import euler_from_quaternion
import math

# temps = ([0]*4)
# goal = ([0.0]*4)
# arm_goal = ([0.0]*4)
# angle = ([0.0]*4)
# arm_angle = ([0.0]*4)
# error = ([0.0]*4)
# arm_error = ([0.0]*4)
# velocity = ([0.0]*4)
# arm_velocity = ([0.0]*4)
# load = ([0.0]*4)
# arm_load = ([0.0]*4)
# moving = ([False]*4)
# voltage = ([0.0]*4)


class SuspensionController:
    def __init__(self):
        print("INIT")
        rospy.init_node('suspension_controller', anonymous=True)
        rospy.on_shutdown(self.on_shutdown)
        
        rospy.wait_for_service('Moving_Status')
        
	self.outputMean = zeros((4, 20))
	self.outputMeanCount = zeros(4)

        self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1) #stesso dei dynamixel
        self.rate = rospy.Rate(40)#self.diagnostics_rate)
        #self.ID = rospy.get_param('/dynamixel/suspension_port/connected_ids')
        #self.nID = len(self.ID)
        self.deltaH_inertial = ([0.0]*4)
        self.deltaH_chassis = ([0.0]*4)
        self.angoli_chassis = ([0.0]*3)
        self.posa_chassis = ([0.0]*3)
        self.angoli_chassis_virtuale = ([0.0]*3)
        self.posa_chassis_virtuale = ([0.0]*3)
        self.altezza = 0.0
        self.range_front = 0.4
        self.range_post = 0.4
        self.deltaH_chassis_virtuale = 0.0
        self.fi = ([0.0]*4)
        self.Z_ruote = ([0.0]*4)
        self.deltaH_hub = ([0.0]*4)
        self.angoli_sosp = ([0.0]*4)
        self.torque = ([0.0]*4)
        self.torquef = ([[0.0]*21]*4)
        self.pos_arm = ([0.0]*4)
        
        self.joint_state_out = JointStateOut()
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        
        self.req_height = 0.25
        self.req_height_temp = 0.25
        self.recovery_height = False
        self.mode = 0
        
        self.pointer = ([0]*4)
        
        self.init_ok = False
        self.init_pos = 0 #0 down to up, 1 up to down
        
        # aggiunta altri parametri richiesti
        
        self.start()
        
        time.sleep(2) # todo mettere metodo più furbo per aspettare messaggi validi dalla scheda adc
        
        #init posizione
        #TODO fare parametrico in base ai dati che arrivano dagli accelerometri circa la posizione dei bracci, se non ricevo dati non fare nulla
        #if self.angoli_sosp[0] < 0.2 && self.angoli_sosp[1] < 0.2 && self.angoli_sosp[2] < 0.2 && self.angoli_sosp[3] < 0.2:
        if False:
            self.init_pos = 0
            self.initializa(0,self.init_pos) # 0 means all
        #elif self.angoli_sosp[0] > 0.9 && self.angoli_sosp[1] > 0.9 && self.angoli_sosp[2] > 0.9 && self.angoli_sosp[3] > 0.9:
        #    self.init_pos = 1
        #    self.initialize(0,self.init_pos)
        #else:
        #    print("Init failed")
        #    TODO STAMPARE ERRORE ed uscire
        #    rospy.signal_shutdown("Init failed")
        #    self.init_ok = True #just in case
        #
        #while !self.init_ok:
        #    for i in range(0,4):
        #       self.init_ok = True
        #        if self.angoli_sosp[i] < 0.2 || self.angoli_sosp[i] > 0.9:
        #           try:
        #                TODO PRINT WARNING
        #                stop = rospy.ServiceProxy('/motore_' + (i+1) + '_controller/set_torque', SetTorque)
        #                resp = stop(False)
        #               self.initialize(i+1,self.init_pos)
        #            except rospy.ServiceException, e:
        #                print("Stop service call failed: %s"%e)
        #                self.init_ok = False
        
        #self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        #if self.diagnostics_rate > 0: Thread(target=self.diagnostics_processor).start()
        
        self.main()


    def on_shutdown(self):
        self.stop
    
    
    def start(self):
        self.running = True
#        for i in range(1,5):
#             self.joint_state_sub = rospy.Subscriber('/motore_'+ str(i) +'_controller/state', JointState, self.process_command,i)
#             self.arm_state_sub = rospy.Subscriber('/motore_'+ str(i) +'_controller/arm/state', JointState, self.process_arm_command,i)
#             self.motor_states_sub = rospy.Subscriber('/motor_states/suspension_port', MotorStateList, self.process_motor_states,i)
             #self.command_pub = rospy.Pubblisher(self.controller_namespace + '/command', Float64)
        self.command_1_pub = rospy.Publisher('/motore_1_controller/command', Float64)
        self.command_arm1_pub = rospy.Publisher('/motore_1_controller/arm/command', Float64)
        self.command_tor1_pub = rospy.Publisher('/motore_1_controller/vel_tor/command', Float64)
        self.command_2_pub = rospy.Publisher('/motore_2_controller/command', Float64)
        self.command_arm2_pub = rospy.Publisher('/motore_2_controller/arm/command', Float64)
        self.command_tor2_pub = rospy.Publisher('/motore_2_controller/vel_tor/command', Float64)
        self.command_3_pub = rospy.Publisher('/motore_3_controller/command', Float64)
        self.command_arm3_pub = rospy.Publisher('/motore_3_controller/arm/command', Float64)
        self.command_tor3_pub = rospy.Publisher('/motore_3_controller/vel_tor/command', Float64)
        self.command_4_pub = rospy.Publisher('/motore_4_controller/command', Float64)
        self.command_arm4_pub = rospy.Publisher('/motore_4_controller/arm/command', Float64)
        self.command_tor4_pub = rospy.Publisher('/motore_4_controller/vel_tor/command', Float64)
        self.joint_state_out_pub = rospy.Publisher('/joint_states', JointStateOut)
        
        self.arm_status_1_sub = rospy.Subscriber('/motore_1_controller/arm/state', JointState, self.process_arm_1)
        self.arm_status_2_sub = rospy.Subscriber('/motore_2_controller/arm/state', JointState, self.process_arm_2) 
        self.arm_status_3_sub = rospy.Subscriber('/motore_3_controller/arm/state', JointState, self.process_arm_3) 
        self.arm_status_4_sub = rospy.Subscriber('/motore_4_controller/arm/state', JointState, self.process_arm_4) 
        
        self.range_front_sub = rospy.Subscriber('/ADC/range_front_down', Range, self.process_range_front)
        self.range_post_sub = rospy.Subscriber('/ADC/range_post_down', Range, self.process_range_post)
 
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #self.IMU_sub = rospy.Subscriber('/imu/data_filtered', Imu, self.process_IMU)
        self.sus_sub = rospy.Subscriber('/ADC/suspension', sosp_Adc, self.process_suspension)
        
        self.service_height = rospy.Service('suspension_controller/set_height', set_height, self.handle_set_height)
        self.service_mode = rospy.Service('suspension_controller/set_mode', set_mode, self.handle_set_mode)
        self.service_stop = rospy.Service('suspension_controller/stop_all', stopAll, self.handle_stopAll)

    def stop(self):
        self.running = False
        #self.joint_state_sub.unregister()
#        self.arm_state_sub.unregister()
#        self.motor_states_sub.unregister()
        self.command_arm1_pub.unregister()
        self.command_tor1_pub.unregister()
        self.command_arm2_pub.unregister()
        self.command_tor2_pub.unregister()
        self.command_arm3_pub.unregister()
        self.command_tor3_pub.unregister()
        self.command_arm4_pub.unregister()
        self.command_tor4_pub.unregister()
        self.joint_state_out_pub.unregister()
        
        self.arm_status_1_sub.unregister()
        self.arm_status_2_sub.unregister()
        self.arm_status_3_sub.unregister()
        self.arm_status_4_sub.unregister()
        
        self.range_front_sub.unregister()
        self.range_post_sub.unregister()
        self.sus_sub.unregister()
        #aggiungere topic adc
        
        
    def handle_set_height(self, req):
        self.req_height = req.height + 0.05
        return [True]


    def handle_set_mode(self, req):
        self.mode = req.mode
        return [True]

    def handle_stopAll(self, req):
        try:
            stop1 = rospy.ServiceProxy('/motore_1_controller/set_torque', SetTorque)
            resp1 = stop1(False)
            stop2 = rospy.ServiceProxy('/motore_2_controller/set_torque', SetTorque)
            resp2 = stop2(False)
            stop3 = rospy.ServiceProxy('/motore_3_controller/set_torque', SetTorque)
            resp3 = stop3(False)
            stop4 = rospy.ServiceProxy('/motore_4_controller/set_torque', SetTorque)
            resp4 = stop4(False)
            return [resp1.response & resp2.response & resp3.response & resp4.response]
        except rospy.ServiceException, e:
            print("Stop service call failed: %s"%e)
        return []

    def initializa(self,id,init_pos):
        print("%d    %d",id,init_pos)
        
        if init_pos == 0:
            ang_p = 6.28
            ang_n = 0.0
        elif init_pos == 1:
            ang_p = 0.0
            ang_n = 6.28
        else:
            return[]
            
        if id == 0 or id == 1:
            self.command_1_pub.publish(ang_p)  #1 positiva
        if id == 0 or id == 2:
            self.command_2_pub.publish(ang_n)  #2 negativa
        if id == 0 or id == 3:
            self.command_3_pub.publish(ang_n)  #3 negativa
        if id == 0 or id == 4:
            self.command_4_pub.publish(ang_p)  #4 positiva
        print("Movimento")
        time.sleep(5.0);
        
        if init_pos == 0:
            ang_p = 0.5
            ang_n = -0.5
        else:
            ang_p = -0.5
            ang_n = 0.5
            
        if id == 0 or id == 1:
            self.command_tor1_pub.publish(ang_p)
        if id == 0 or id == 2:
            self.command_tor2_pub.publish(ang_n)
        if id == 0 or id == 3:
            self.command_tor3_pub.publish(ang_n)
        if id == 0 or id == 4:
            self.command_tor4_pub.publish(ang_p)
        print("Step")
        time.sleep(4.0);
        
        if init_pos == 0:
            ang_p = 3.14
            ang_n = -3.14
        else:
            ang_p = -3.14
            ang_n = 3.14
        
        print("Bloccaggio")
        for i in range(0,7):
            if id == 0 or id == 1:
                self.command_1_pub.publish(ang_p)
            if id == 0 or id == 2:
                self.command_2_pub.publish(ang_n)
            if id == 0 or id == 3:
                self.command_3_pub.publish(ang_n)
            if id == 0 or id == 4:
                self.command_4_pub.publish(ang_p)
            time.sleep(0.1);
        

    def diagnostics_processor(self):
#        diag_msg = DiagnosticArray()
#        
#        rate = rospy.Rate(self.diagnostics_rate)
#        while not rospy.is_shutdown():
#            diag_msg.status = []
#            diag_msg.header.stamp = rospy.Time.now()
#            
#            for controller in self.controllers.values():
#                try:
#                    joint_state = controller.joint_state
#                    temps = joint_state.motor_temps
#                    max_temp = max(temps)
#                    
#                    status = DiagnosticStatus()
#                    status.name = 'Joint Controller (%s)' % controller.joint_name
#                    status.hardware_id = 'Robotis Dynamixel %s on port %s' % (str(joint_state.motor_ids), controller.port_namespace)
#                    status.values.append(KeyValue('Goal', str(joint_state.goal_pos)))
#                    status.values.append(KeyValue('Position', str(joint_state.current_pos)))
#                    status.values.append(KeyValue('Error', str(joint_state.error)))
#                    status.values.append(KeyValue('Velocity', str(joint_state.velocity)))
#                    status.values.append(KeyValue('Load', str(joint_state.load)))
#                    status.values.append(KeyValue('Moving', str(joint_state.is_moving)))
#                    status.values.append(KeyValue('Temperature', str(max_temp)))
#                    status.level = DiagnosticStatus.OK
#                    status.message = 'OK'
#                        
#                    diag_msg.status.append(status)
#                except:
#                    pass
#                    
#            self.diagnostics_pub.publish(diag_msg)
            self.rate.sleep()


    def process_range_front(self, msg):
        self.range_front = msg.range
        
    def process_range_post(self, msg):
        self.range_post = msg.range

    def process_arm_1(self, msg):
        self.torquef[0][self.pointer[0]] = msg.load
        if self.pointer[0] == 19: self.pointer[0] = 0
        else: self.pointer[0] += 1
        self.torquef[0][20] = 0.0
        for i in range(0,20):
           self.torquef[0][20] = self.torquef[0][20]+ self.torquef[0][i]
        self.torquef[0][20] /= 20.0
        
        self.torque[0] = self.torquef[0][20]
        self.pos_arm[0] = msg.current_pos
        
    def process_arm_2(self, msg):
        self.torquef[1][self.pointer[1]] = msg.load
        if self.pointer[1] == 19: self.pointer[1] = 0
        else: self.pointer[1] += 1
        self.torquef[1][20] = 0.0
        for i in range(0,20):
           self.torquef[0][20] = self.torquef[1][20]+ self.torquef[1][i]
        self.torquef[1][20] /= 20.0
        
        self.torque[1] = self.torquef[1][20]
        self.pos_arm[1] = msg.current_pos
        
    def process_arm_3(self, msg):
        self.torquef[2][self.pointer[2]] = msg.load
        if self.pointer[2] == 19: self.pointer[2] = 0
        else: self.pointer[2] += 1
        self.torquef[2][20] = 0.0
        for i in range(0,20):
           self.torquef[0][20] = self.torquef[2][20]+ self.torquef[2][i]
        self.torquef[2][20] /= 20.0
        
        self.torque[2] = self.torquef[2][20]
        self.pos_arm[2] = msg.current_pos
        
    def process_arm_4(self, msg):
        self.torquef[3][self.pointer[3]] = msg.load
        if self.pointer[3] == 19: self.pointer[3] = 0
        else: self.pointer[3] += 1
        self.torquef[3][20] = 0.0
        for i in range(0,20):
           self.torquef[3][20] = self.torquef[3][20]+ self.torquef[3][i]
        self.torquef[3][20] /= 20.0
        
        self.torque[3] = self.torquef[3][20]
        self.pos_arm[3] = msg.current_pos
        
    def process_suspension(self, msg):
        self.angoli_sosp[0] = msg.sosp1
        self.angoli_sosp[1] = msg.sosp2
        self.angoli_sosp[2] = msg.sosp3
        self.angoli_sosp[3] = msg.sosp4


#    def process_arm_command(self, msg, ID):
        #ID = int(msg.motor_ids)
#        arm_goal[ID] = msg.goal_pos
#        arm_angle[ID] = msg.current_pos
#        arm_error[ID] = msg.error
#        arm_velocity[ID] = msg.velocity
#        arm_load[ID] = msg.load
#        self.printa()
        #print(rospy.get_name() + ": I heard %s" % arm_angle)
        
#    def process_motor_states(self, state_list, ID):
#        if self.running:
#            state = filter(lambda state: state.id == ID, state_list.motor_states)
#            if state:
#                state = state[0]
#                voltage[ID] = state.voltage


#    def printa(self):
        #print(str(angle[1]) + " " +  str(angle[2]) + " " +  str(angle[3]) + " " +  str(angle[4]))
#        print(str(voltage[1]) + " " +  str(voltage[2]) + " " +  str(voltage[3]) + " " +  str(voltage[4]))


    def get_tf(self): 
        for i in range(0,4):
           try:
               if i == 0:
                   (trans,rot) = self.listener.lookupTransform('chassis', 'wheel_f_l', rospy.Time(0))
               elif i == 1:
                   (trans,rot) = self.listener.lookupTransform('chassis', 'wheel_p_l', rospy.Time(0))
               elif i == 2:
                   (trans,rot) = self.listener.lookupTransform('chassis', 'wheel_f_r', rospy.Time(0))
               elif i == 3:
                   (trans,rot) = self.listener.lookupTransform('chassis', 'wheel_p_r', rospy.Time(0))
               self.deltaH_chassis[i]= -trans[2] + 0.090
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               print('no_info')
               self.deltaH_chassis[i]= 0.0
               continue
           #print('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #print('rotation: ',[(180.0/math.pi)*i for i in angles])
        print('distanza ruote da chassis: ',self.deltaH_chassis)        
        
        
        for i in range(0,4):
           try:
               if i == 0:
                   (trans,rot) = self.listener.lookupTransform('inertial', 'wheel_f_l', rospy.Time(0))
               elif i == 1:
                   (trans,rot) = self.listener.lookupTransform('inertial', 'wheel_p_l', rospy.Time(0))
               elif i == 2:
                   (trans,rot) = self.listener.lookupTransform('inertial', 'wheel_f_r', rospy.Time(0))
               elif i == 3:
                   (trans,rot) = self.listener.lookupTransform('inertial', 'wheel_p_r', rospy.Time(0))
               self.deltaH_inertial[i]= -trans[2] + 0.090
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               print('no_info')
               self.deltaH_inertial[i]= 0.0
               continue
           #print('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #print('rotation: ',[(180.0/math.pi)*i for i in angles])
        print('distanza ruote da inertial: ',self.deltaH_inertial)
        self.br.sendTransform( ( 0.0, 0.0, max(self.deltaH_inertial) ), (0.0, 0.0, 0.0, 1.0) , rospy.Time.now(), 'inertial','base_link');
        
        
        try:
            (trans,rot) = self.listener.lookupTransform('base_link', 'chassis', rospy.Time(0))
            print('posizione chassis: ',trans)
            angles = euler_from_quaternion(rot)
            #print('rotation: ',[(180.0/math.pi)*i for i in angles])
            print('rotazione chassis: ',angles)
            self.altezza=min(0.05 + self.range_front*math.cos(math.fabs(angles[1]))*math.cos(math.fabs(angles[0])),0.05 + self.range_post*math.cos(math.fabs(angles[1]))*math.cos(math.fabs(angles[0])),min(self.deltaH_chassis))
            print('altezza chassis corretta da range: ', self.altezza -0.05)
            print('altezza richiesta: ',self.req_height -0.05)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('no_info') #continue
            angles = [0,0,0]
            trans = [0,0,0]
            self.altezza = 0
        self.angoli_chassis = angles
        self.posa_chassis = trans
        
        
        for i in range(0,4):
           try:
               if i == 0:
                   (trans,rot) = self.listener.lookupTransform('base_link', 'wheel_f_l', rospy.Time(0))
               elif i == 1:
                   (trans,rot) = self.listener.lookupTransform('base_link', 'wheel_p_l', rospy.Time(0))
               elif i == 2:
                   (trans,rot) = self.listener.lookupTransform('base_link', 'wheel_f_r', rospy.Time(0))
               elif i == 3:
                   (trans,rot) = self.listener.lookupTransform('base_link', 'wheel_p_r', rospy.Time(0))
               self.Z_ruote[i]= trans[2]
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               print('no_info')
               self.Z_ruote[i]= 0.0
               continue
           #print('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #print('rotation: ',[(180.0/math.pi)*i for i in angles])
        print('distanza ruote da base_link: ',self.Z_ruote)
        
        
        self.deltaH_chassis_virtuale = self.req_height -  self.altezza
        print('distanza da inertial a chassis virtuale: ',self.deltaH_chassis_virtuale)
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        self.joint_state_out.name.append("delta_h")
        self.joint_state_out.position.append(self.deltaH_chassis_virtuale)
        self.joint_state_out.header.stamp = rospy.Time.now()  
        self.joint_state_out_pub.publish(self.joint_state_out)
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        self.joint_state_out.name.append("rpy_r")
        self.joint_state_out.position.append(-self.angoli_chassis[0])
        self.joint_state_out.header.stamp = rospy.Time.now()  
        self.joint_state_out_pub.publish(self.joint_state_out)
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        self.joint_state_out.name.append("rpy_y")
        self.joint_state_out.position.append(-self.angoli_chassis[1])
        self.joint_state_out.header.stamp = rospy.Time.now()  
        self.joint_state_out_pub.publish(self.joint_state_out)
        
        
        try:
            (trans,rot) = self.listener.lookupTransform('base_link', 'chassis_virtuale', rospy.Time(0))
            print(  'posizione chassis_virtuale: ',trans)
            angles = euler_from_quaternion(rot)
            #print('rotation: ',[(180.0/math.pi)*i for i in angles])
            print('rotazione chassis_virtuale: ',angles)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('no_info') #continue)
            angles = [0,0,0]
            trans = [0,0,0]
        self.angoli_chassis_virtuale = angles
        self.posa_chassis_virtuale = trans
        
        
#        for i in range(0,4):
#           try:
#               if i == 0:
#                   (trans,rot) = self.listener.lookupTransform('inertial', 'leg_f_l', rospy.Time(0))
#               elif i == 1:
#                   (trans,rot) = self.listener.lookupTransform('inertial', 'leg_p_l', rospy.Time(0))
#               elif i == 2:
#                   (trans,rot) = self.listener.lookupTransform('inertial', 'leg_f_r', rospy.Time(0))
#               elif i == 3:
#                   (trans,rot) = self.listener.lookupTransform('inertial', 'leg_p_r', rospy.Time(0))
#               self.deltaH_hub[i]= -trans[2]
#               print('distanza hub da inertial: ',self.deltaH_hub)
#           except (tf.LookupException, tf.ConnectivityException):
#               print('no_info')
#               self.deltaH_hub[i]= 0.0
#               continue


    def calculate_fi(self):
        for i in range(0,4):
            e = []
            try:
               self.fi[i]=math.acos(( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20)
            except (ValueError):
               print('out_of_range')
               if (( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20) > 1:
                  self.fi[i]=0.18
               elif (( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20) < 0:
                  self.fi[i]=0.96
               continue
            self.joint_state_out.name = []
            self.joint_state_out.position = []
            self.joint_state_out.velocity = []
            self.joint_state_out.effort = []
            if i == 0:
                self.joint_state_out.name.append("hub_f_l_virtuale")
            elif i == 1:
                self.joint_state_out.name.append("hub_p_l_virtuale")
            elif i == 2:
                self.joint_state_out.name.append("hub_f_r_virtuale")
            elif i == 3:
                self.joint_state_out.name.append("hub_p_r_virtuale")
            self.joint_state_out.position.append(self.fi[i])
            self.joint_state_out.header.stamp = rospy.Time.now()  
            self.joint_state_out_pub.publish(self.joint_state_out)
        print('angolo virtuale: ',self.fi)
        print('---------------')
        
        
    def output_fi(self):
        self.recovery = 0
	divisor = 5
        for i in range(0,4): 
            phi = self.fi[i]
            if phi < 0.19: 
                 phi = 0.19
                 self.recovery += 1
            elif phi > 0.95: 
                 phi = 0.95
                 self.recovery += 1
	    publish = False
	    self.outputMean[i][self.outputMeanCount[i]] += phi
	    self.outputMeanCount[i] += 1
	    if self.outputMeanCount[i] >= divisor:
		publish = True
		phi = sum(self.outputMean) / divisor
		self.outputMean[i] = zeros(divisor)
		self.outputMeanCount[i] = 0
		
                  
	    if publish:
		    if i == 0:
			self.command_arm1_pub.publish(phi)
		    elif i == 1:
			self.command_arm2_pub.publish(phi)
		    elif i == 2:
			self.command_arm3_pub.publish(phi)
		    elif i == 3:
			self.command_arm4_pub.publish(phi)
            
#        if self.recovery == 3:        
#                 print('attivata recovery altezza')
#                 print('---------------')
 #                if not self.recovery_height: #prima attivazione
#                    self.recovery_height = True
#                    self.req_height_temp = self.req_height
#                 if self.req_height >= 0.061:
#                    self.req_height = self.req_height - 0.001
#                 else: #successive
#                    if self.req_height >= 0.061:
#                        self.req_height = self.req_height - 0.001
            
#        elif self.req_height <= self.req_height_temp and self.recovery_height: #non sono pi\F9 in recovery e devo correggere altezza
#                self.req_height = self.req_height + 0.001
#                if self.req_height_temp == self.req_height:
#                   self.recovery_height = False
#                   print('rientro recovery altezza')
#                   print('---------------')

    def pull_down(self):
        print('coppie',self.torque)
        for i in range(0,4): 
            while self.torque[i] < -0.1:
                phi = self.pos_arm[i] - 0.01
                if phi < 0.19: 
                     phi = 0.19
                elif phi > 0.95: 
                     phi = 0.95
                      
                if i == 0:
                    self.command_arm1_pub.publish(phi)
                elif i == 1:
                    self.command_arm2_pub.publish(phi)
                elif i == 2:
                    self.command_arm3_pub.publish(phi)
                elif i == 3:
                    self.command_arm4_pub.publish(phi)
            
                self.joint_state_out.name = []
                self.joint_state_out.position = []
                self.joint_state_out.velocity = []
                self.joint_state_out.effort = []
                if i == 0:
                    self.joint_state_out.name.append("hub_f_l_virtuale")
                elif i == 1:
                    self.joint_state_out.name.append("hub_p_l_virtuale")
                elif i == 2:
                    self.joint_state_out.name.append("hub_f_r_virtuale")
                elif i == 3:
                    self.joint_state_out.name.append("hub_p_r_virtuale")
                self.joint_state_out.position.append(phi)
                self.joint_state_out.header.stamp = rospy.Time.now()  
                self.joint_state_out_pub.publish(self.joint_state_out)
                
                time.sleep(0.1)
            
            print('recovery ruota sollevata')
            print('---------------')
        
    def main(self):    
        print("TF")
        
        count = 0
        
        #rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            
            if self.mode == 0: # solo simulazione
                self.get_tf()
                self.calculate_fi()
                #print("coppia",self.torque)
            elif self.mode == 1: # solo inseguitore
                #TODO a step di entit prop all'errore e limitato nel range
                continue
            elif self.mode == 2: # solo SIL
                self.get_tf()
                self.calculate_fi()
                self.output_fi()
            elif self.mode == 3: # SIL + anti sollevamento
                self.pull_down()
                self.get_tf()
                self.calculate_fi()
                self.output_fi()
                
            #TODO algoritmo suddivisione carico
            
            #TODO algoritmo anti sollevamento
            
            #TODO algortimo bloccante superamento ostacoli
            
            #TODO da sincronizzare poi con i motori
            # if count%50 == 0:
                # try:
                    # add_two_ints = rospy.ServiceProxy('Moving_Status', isMoving)
                    # resp1 = Moving_Status(False)
                    # return resp1
                # except rospy.ServiceException, e:
                    # print "Service call failed: %s"%e
                    
                # time.sleep(0.500)
                
                # try:
                    # add_two_ints = rospy.ServiceProxy('Moving_Status', isMoving)
                    # resp1 = Moving_Status(True)
                    # return resp1
                # except rospy.ServiceException, e:
                    # print "Service call failed: %s"%e
                
                
                
            self.rate.sleep()



if __name__ == '__main__':
    try:
        SuspensionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

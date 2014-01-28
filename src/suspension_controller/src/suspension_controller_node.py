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
from suspension_controller.srv import freeze
from dynamixel_controllers.srv import SetTorque
from adc.srv import movingService

import sys

import time

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from adc.msg import sosp_Adc
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as JointStateOut
from suspension_controller.msg import STATUS_ASM

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
        rospy.init_node('suspension_controller', anonymous=True, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.on_shutdown)
        
        #rospy.wait_for_service('Moving_Status')
        
        self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1) #stesso dei dynamixel
        self.speed = rospy.get_param('/motore_1_controller/joint_speed', 0.5) #dovrebbe essere uguale per tutti
        self.hz = 40
        self.rate = rospy.Rate(self.hz)#self.diagnostics_rate)
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
        self.delta = ([0.0]*4)
        self.Z_ruote = ([0.0]*4)
        self.deltaH_hub = ([0.0]*4)
        self.angoli_sosp = ([0.0]*4)
        self.torque = ([0.0]*4)

        self.torquef = []
        for i in range(4):
           self.torquef.append([0]*21)
        
        self.pos_arm = ([0.0]*4)
        self.error_arm = ([0.0]*4)
        self.motor_temp = ([0.0]*4)
        
        self.pull_down_sts = ([False]*4)
        self.out_of_range_sts = ([False]*4)
        
        self.joint_state_out = JointStateOut()
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        
        self.status_asm = STATUS_ASM()
        
        self.req_height = 0.25
        self.req_height_temp = 0.25
        self.recovery_height = False
        self.mode = 0
        self.freeze = False
        
        self.pointer = ([0]*4)
        
        self.init_ok = False
        self.init_pos = 0 #0 down to up, 1 up to down
        
        # aggiunta altri parametri richiesti
        
        self.start()
        
        rospy.loginfo("INIT")
        
        time.sleep(2) # todo mettere metodo piu' furbo per aspettare messaggi validi dalla scheda adc
        
        #init posizione
        #TODO fare parametrico in base ai dati che arrivano dagli accelerometri circa la posizione dei bracci, se non ricevo dati non fare nulla
        #if self.angoli_sosp[0] < 0.2 && self.angoli_sosp[1] < 0.2 && self.angoli_sosp[2] < 0.2 && self.angoli_sosp[3] < 0.2:
        if False:
            rospy.loginfo("Start position: sollevato")
            self.init_pos = 0
            self.initializa(0,self.init_pos) # 0 means all
        #elif self.angoli_sosp[0] > 0.9 && self.angoli_sosp[1] > 0.9 && self.angoli_sosp[2] > 0.9 && self.angoli_sosp[3] > 0.9:
        # rospy.loginfo("Start position: a terra")
        # self.init_pos = 1
        # self.initialize(0,self.init_pos)
        #else:
        # rospy.logfatal("Init failed")
        # TODO STAMPARE ERRORE ed uscire
        # rospy.signal_shutdown("Init failed")
        # self.init_ok = True #just in case
        #
        #while !self.init_ok:
        # for i in range(0,4):
        # self.init_ok = True
        # if self.angoli_sosp[i] < 0.2 || self.angoli_sosp[i] > 0.9:
        # try:
        # rospy.logwarning("Avvio fallito su %i, retry",i)
        # stop = rospy.ServiceProxy('/motore_' + (i+1) + '_controller/set_torque', SetTorque)
        # resp = stop(False)
        # self.initialize(i+1,self.init_pos)
        # except rospy.ServiceException, e:
        # rospy.logerror("Stop service call failed: %s"%e)
        # self.init_ok = False
        
        #self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        #if self.diagnostics_rate > 0: Thread(target=self.diagnostics_processor).start()
        
        self.main()


    def on_shutdown(self):
        self.stop
    
    
    def start(self):
        self.running = True
# for i in range(1,5):
# self.joint_state_sub = rospy.Subscriber('/motore_'+ str(i) +'_controller/state', JointState, self.process_command,i)
# self.arm_state_sub = rospy.Subscriber('/motore_'+ str(i) +'_controller/arm/state', JointState, self.process_arm_command,i)
# self.motor_states_sub = rospy.Subscriber('/motor_states/suspension_port', MotorStateList, self.process_motor_states,i)
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
        
        self.status_asm_pub = rospy.Publisher('/status_asm', STATUS_ASM)
        
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
        self.service_freeze = rospy.Service('suspension_controller/freeze', freeze, self.handle_freeze)

    def stop(self):
        self.running = False
        #self.joint_state_sub.unregister()
# self.arm_state_sub.unregister()
# self.motor_states_sub.unregister()
        self.command_arm1_pub.unregister()
        self.command_tor1_pub.unregister()
        self.command_arm2_pub.unregister()
        self.command_tor2_pub.unregister()
        self.command_arm3_pub.unregister()
        self.command_tor3_pub.unregister()
        self.command_arm4_pub.unregister()
        self.command_tor4_pub.unregister()
        self.joint_state_out_pub.unregister()
        self.status_asm_pub.unregister()
        
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
        rospy.loginfo("Altezza settata a %f", req.height)
        return [True]


    def handle_set_mode(self, req):
        self.mode = req.mode
        response = []
        if req.mode == 0:
            self.delta = ([0.0]*4)
            response = "Simulazione"
        elif req.mode == 1:
            self.pull_down_sts = ([False]*4)
            response = "Inseguitore"
        elif req.mode == 2:
            self.delta = ([0.0]*4)
            self.pull_down_sts = ([False]*4)
            response = "Osservatore"
        elif req.mode == 3:
            self.delta = ([0.0]*4)
            response = "Osservatore + antisollevamento"
        elif req.mode == 4:
            self.pull_down_sts = ([False]*4)
            response = "Osservatore + inseguitore"
        elif req.mode == 5:
            response = "Osservatore + inseguitore + antisollevamento"
        else:
            response = "Not supported"
            self.mode = 0
        rospy.loginfo("Mode set to %i", self.mode)
        return [response]

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
            rospy.loginfo("Sospensioni rilasciate")
            return [resp1.response & resp2.response & resp3.response & resp4.response]
        except rospy.ServiceException, e:
            rospy.logerror("Stop service call failed: %s"%e)
        return []
        
    def handle_freeze(self, req):
        self.freeze = req.freeze
        rospy.loginfo("Richiesta freeze %b", self.freeze)
        if self.freeze:
            self.get_tf()
            self.calculate_fi()
            self.delta = ([0.0]*4)
            self.output_fi()
            while min(self.torque) < 0.1 :
                self.pull_down()
        return [self.freeze]

    def initializa(self,id,init_pos):
        rospy.loginfo("Inizializza ruota %d (0 = tutte) da posizione %d (0 alzato, 1 a terra)",id,init_pos)
        
        if init_pos == 0:
            ang_p = 6.28
            ang_n = 0.0
        elif init_pos == 1:
            ang_p = 0.0
            ang_n = 6.28
        else:
            return[]
            
        if id == 0 or id == 1:
            self.command_1_pub.publish(ang_p) #1 positiva
        if id == 0 or id == 2:
            self.command_2_pub.publish(ang_n) #2 negativa
        if id == 0 or id == 3:
            self.command_3_pub.publish(ang_n) #3 negativa
        if id == 0 or id == 4:
            self.command_4_pub.publish(ang_p) #4 positiva
        rospy.loginfo("Movimento")
        time.sleep(5.0);
        
        if init_pos == 0:
            ang_p = 0.5 #una velocita'!
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
        rospy.loginfo("Step")
        time.sleep(4.0);
        
        if init_pos == 0:
            ang_p = 3.14
            ang_n = -3.14
        else:
            ang_p = -3.14
            ang_n = 3.14
        
        rospy.loginfo("Bloccaggio")
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
# diag_msg = DiagnosticArray()
#
# rate = rospy.Rate(self.diagnostics_rate)
# while not rospy.is_shutdown():
# diag_msg.status = []
# diag_msg.header.stamp = rospy.Time.now()
#
# for controller in self.controllers.values():
# try:
# joint_state = controller.joint_state
# temps = joint_state.motor_temps
# max_temp = max(temps)
#
# status = DiagnosticStatus()
# status.name = 'Joint Controller (%s)' % controller.joint_name
# status.hardware_id = 'Robotis Dynamixel %s on port %s' % (str(joint_state.motor_ids), controller.port_namespace)
# status.values.append(KeyValue('Goal', str(joint_state.goal_pos)))
# status.values.append(KeyValue('Position', str(joint_state.current_pos)))
# status.values.append(KeyValue('Error', str(joint_state.error)))
# status.values.append(KeyValue('Velocity', str(joint_state.velocity)))
# status.values.append(KeyValue('Load', str(joint_state.load)))
# status.values.append(KeyValue('Moving', str(joint_state.is_moving)))
# status.values.append(KeyValue('Temperature', str(max_temp)))
# status.level = DiagnosticStatus.OK
# status.message = 'OK'
#
# diag_msg.status.append(status)
# except:
# pass
#
# self.diagnostics_pub.publish(diag_msg)
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
        self.error_arm[0] = msg.error
        self.motor_temp[0] = int(msg.motor_temps[0])
        
    def process_arm_2(self, msg):
        self.torquef[1][self.pointer[1]] = msg.load
        if self.pointer[1] == 19: self.pointer[1] = 0
        else: self.pointer[1] += 1
        self.torquef[1][20] = 0.0
        for i in range(0,20):
           self.torquef[1][20] = self.torquef[1][20]+ self.torquef[1][i]
        self.torquef[1][20] /= 20.0
        
        self.torque[1] = self.torquef[1][20]
        self.pos_arm[1] = msg.current_pos
        self.error_arm[1] = msg.error
        self.motor_temp[1] = int(msg.motor_temps[0])
        
    def process_arm_3(self, msg):
        self.torquef[2][self.pointer[2]] = msg.load
        if self.pointer[2] == 19: self.pointer[2] = 0
        else: self.pointer[2] += 1
        self.torquef[2][20] = 0.0
        for i in range(0,20):
           self.torquef[2][20] = self.torquef[2][20]+ self.torquef[2][i]
        self.torquef[2][20] /= 20.0
        
        self.torque[2] = self.torquef[2][20]
        self.pos_arm[2] = msg.current_pos
        self.error_arm[2] = msg.error
        self.motor_temp[2] = int(msg.motor_temps[0])
        
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
        self.error_arm[3] = msg.error
        self.motor_temp[3] = int(msg.motor_temps[0])
        
    def process_suspension(self, msg):
        self.angoli_sosp[0] = msg.sosp1
        self.angoli_sosp[1] = msg.sosp2
        self.angoli_sosp[2] = msg.sosp3
        self.angoli_sosp[3] = msg.sosp4



    def get_tf(self):
        rospy.loginfo("get TF")
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
               rospy.logwarn('no_info chassis to wheel %i', i+1)
               self.deltaH_chassis[i]= 0.0
               continue
           #rospy.logdebug('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
        rospy.logdebug("distanza ruote da chassis: %f %f %f %f",self.deltaH_chassis[0],self.deltaH_chassis[1],self.deltaH_chassis[2],self.deltaH_chassis[3])
        
        
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
               rospy.logwarn("no_info inertial to wheel %i", i+1)
               self.deltaH_inertial[i]= 0.0
               continue
           #rospy.logdebug('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
        rospy.logdebug("distanza ruote da inertial: %f %f %f %f",self.deltaH_inertial[0],self.deltaH_inertial[1],self.deltaH_inertial[2],self.deltaH_inertial[3])
        self.br.sendTransform( ( 0.0, 0.0, max(self.deltaH_inertial) ), (0.0, 0.0, 0.0, 1.0) , rospy.Time.now(), 'inertial','base_link');
        
        
        try:
            (trans,rot) = self.listener.lookupTransform('base_link', 'chassis', rospy.Time(0))
            rospy.logdebug("posizione chassis: %f %f %f",trans[0],trans[1],trans[2])
            angles = euler_from_quaternion(rot)
            #rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
            rospy.logdebug("rotazione chassis: %f %f %f",angles[0],angles[1],angles[2])
            self.altezza=min(0.05 + self.range_front*math.cos(math.fabs(angles[1]))*math.cos(math.fabs(angles[0])),0.05 + self.range_post*math.cos(math.fabs(angles[1]))*math.cos(math.fabs(angles[0])),min(self.deltaH_chassis))
            rospy.logdebug("altezza chassis corretta da range: %f", self.altezza -0.05)
            rospy.logdebug("altezza richiesta: %f",self.req_height -0.05)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info base_link to chassis") #continue
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
               rospy.logwarn("no_info base_link to wheel %i",i+1)
               self.Z_ruote[i]= 0.0
               continue
           #rospy.logdebug('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
        rospy.logdebug("distanza ruote da base_link: %f %f %f %f",self.Z_ruote[0],self.Z_ruote[1],self.Z_ruote[2],self.Z_ruote[3])
        
        
        self.deltaH_chassis_virtuale = self.req_height - self.altezza
        rospy.logdebug("distanza da inertial a chassis virtuale: %f",self.deltaH_chassis_virtuale)
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
            rospy.logdebug("posizione chassis_virtuale: %f %f %f",trans[0],trans[1],trans[2])
            angles = euler_from_quaternion(rot)
            #rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
            rospy.logdebug("angoli chassis_virtuale: %f %f %f",angles[0],angles[1],angles[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('no_info base_link toi chassis_virtuale') #continue)
            angles = [0,0,0]
            trans = [0,0,0]
        self.angoli_chassis_virtuale = angles
        self.posa_chassis_virtuale = trans
        
        
# for i in range(0,4):
# try:
# if i == 0:
# (trans,rot) = self.listener.lookupTransform('inertial', 'leg_f_l', rospy.Time(0))
# elif i == 1:
# (trans,rot) = self.listener.lookupTransform('inertial', 'leg_p_l', rospy.Time(0))
# elif i == 2:
# (trans,rot) = self.listener.lookupTransform('inertial', 'leg_f_r', rospy.Time(0))
# elif i == 3:
# (trans,rot) = self.listener.lookupTransform('inertial', 'leg_p_r', rospy.Time(0))
# self.deltaH_hub[i]= -trans[2]
# rospy.logdebug('distanza hub da inertial: ',self.deltaH_hub)
# except (tf.LookupException, tf.ConnectivityException):
# rospy.logwarn('no_info')
# self.deltaH_hub[i]= 0.0
# continue


    def calculate_fi(self):
        step = 0.0
        roll = 0.0
        pitch = 0.0
        roll = self.angoli_chassis[0]
        pitch = self.angoli_chassis[1]
        
        limit = 0.1
        
        if math.fabs(pitch) > limit or math.fabs(roll) or math.fabs(self.deltaH_chassis_virtuale)> limit:
            for i in range(0,4):
                try:
                   self.fi[i]=math.acos(( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20)
                except (ValueError):
                   rospy.logwarn("out_of_range on wheel %i",i+1)
                   if (( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20) > 1:
                      self.fi[i]=0.39
                   elif (( self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote[i])/0.20) < 0:
                      self.fi[i]=1.22
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
                self.joint_state_out.position.append(self.fi[i] + self.delta[i])
                self.joint_state_out.header.stamp = rospy.Time.now()
                self.joint_state_out_pub.publish(self.joint_state_out)
                
        rospy.loginfo("angolo virtuale: %f %f %f %f",self.fi[0],self.fi[1],self.fi[2],self.fi[3])
        
        
    def output_fi(self):
        #self.recovery = 0
        for i in range(0,4):
            if (self.pull_down_sts[i] == False):
                phi = self.fi[i] + self.delta[i]
                if phi < 0.40:
                     phi = 0.40
                     self.out_of_range_sts[i] = True
                     #self.recovery += 1
                elif phi > 1.21:
                     phi = 1.21
                     self.out_of_range_sts[i] = True
                     #self.recovery += 1
                else:
                    self.out_of_range_sts[i] = False
                
                if self.torque[i] > 3.9:
                    rospy.logwarn("Limite coppia su %i!", i+1)
                    phi = phi + 0.1
                
                if i == 0:
                    self.command_arm1_pub.publish(phi)
                elif i == 1:
                    self.command_arm2_pub.publish(phi)
                elif i == 2:
                    self.command_arm3_pub.publish(phi)
                elif i == 3:
                    self.command_arm4_pub.publish(phi)
                    
                if self.torque[i] > 3.9:
                    time.sleep(0.5)

# if self.recovery == 3:
# print('attivata recovery altezza')
# print('---------------')
 # if not self.recovery_height: #prima attivazione
# self.recovery_height = True
# self.req_height_temp = self.req_height
# if self.req_height >= 0.061:
# self.req_height = self.req_height - 0.001
# else: #successive
# if self.req_height >= 0.061:
# self.req_height = self.req_height - 0.001
            
# elif self.req_height <= self.req_height_temp and self.recovery_height: #non sono pi\F9 in recovery e devo correggere altezza
# self.req_height = self.req_height + 0.001
# if self.req_height_temp == self.req_height:
# self.recovery_height = False
# print('rientro recovery altezza')
# print('---------------')

    def pull_down(self):
        rospy.logdebug("Coppie: %f %f %f %f",self.torque[0],self.torque[1],self.torque[2],self.torque[3])
        if min(self.torque) < 0.1:
            rospy.loginfo('Recovery ruota sollevata')
            for i in range(0,4):
                if self.torque[i] < 0.1:
                    self.pull_down_sts[i] = True
                    phi = self.pos_arm[i] - 0.5
                    if phi < 0.40:
                        phi = 0.40
                    elif phi > 1.21:
                        phi = 1.00
                    
                    rospy.logdebug("Coppia motore %i %f -> comando posizione %f",i+1,self.torque[i],phi)
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
                else:
                    self.pull_down_sts[i] = False
            time.sleep(0.1)
        else:  
            self.pull_down_sts = ([False]*4)

    def follower(self):
        step = 0.0
        step = self.speed/float(self.hz)
        
        limit = 0.03
        ok = False
        
        # nota convenzioni: self.status_asm.mot_pos_1 = self.pos_arm[0] + self.error_arm[0]
        
        if self.error_arm[0] > limit:
            self.delta[0] = self.error_arm[0]
        elif self.error_arm[0] < -limit:
            self.delta[0] = self.error_arm[0]
        else:
            ok = True
            
        if self.error_arm[1] > limit:
            self.delta[1] = self.error_arm[1]
        elif self.error_arm[1] < -limit:
            self.delta[1] = self.error_arm[1]
        else:
            ok = True
            
        if self.error_arm[2] > limit:
            self.delta[2] = self.error_arm[2]
        elif self.error_arm[2] < -limit:
            self.delta[2] = self.error_arm[2]
        else:
            ok = True
            
        if self.error_arm[3] > limit:
            self.delta[3] = self.error_arm[3]
        elif self.error_arm[0] < -limit:
            self.delta[3] = self.error_arm[3]
        else:
            ok = True

        rospy.loginfo("Delta follower: %f %f %f %f",self.delta[0],self.delta[1],self.delta[2],self.delta[3])
            
    
    def set_status(self):
        self.status_asm.pos_1 = self.pos_arm[0]
        self.status_asm.pos_2 = self.pos_arm[1]
        self.status_asm.pos_3 = self.pos_arm[2]
        self.status_asm.pos_4 = self.pos_arm[3]
        self.status_asm.mot_pos_1 = self.pos_arm[0] + self.error_arm[0]
        self.status_asm.mot_pos_2 = self.pos_arm[1] + self.error_arm[1]
        self.status_asm.mot_pos_3 = self.pos_arm[2] + self.error_arm[2]
        self.status_asm.mot_pos_4 = self.pos_arm[3] + self.error_arm[3]
        self.status_asm.command_1 = self.fi[0]
        self.status_asm.command_2 = self.fi[1]
        self.status_asm.command_3 = self.fi[2]
        self.status_asm.command_4 = self.fi[3]
        self.status_asm.delta_1 = self.fi[0] + self.delta[0]
        self.status_asm.delta_2 = self.fi[1] + self.delta[1]
        self.status_asm.delta_3 = self.fi[2] + self.delta[2]
        self.status_asm.delta_4 = self.fi[3] + self.delta[3]
        
        self.status_asm.height_1 = self.Z_ruote[0]
        self.status_asm.height_2 = self.Z_ruote[1]
        self.status_asm.height_3 = self.Z_ruote[2]
        self.status_asm.height_4 = self.Z_ruote[3]
        
        self.status_asm.current_height = self.altezza - 0.05
        self.status_asm.command_height = self.req_height - 0.05
        
        self.status_asm.roll = self.angoli_chassis[0]
        self.status_asm.pitch = self.angoli_chassis[1]
        #TODO inserire is_moving ed abilitare il reset del filtro
        
        #TODO inserire temperature e lettura delle stesse
        
        self.status_asm.motor_1_temp = self.motor_temp[0]
        self.status_asm.motor_2_temp = self.motor_temp[1]
        self.status_asm.motor_3_temp = self.motor_temp[2]
        self.status_asm.motor_4_temp = self.motor_temp[3]
        
        self.status_asm.load_1 = self.torque[0]
        self.status_asm.load_2 = self.torque[1]
        self.status_asm.load_3 = self.torque[2]
        self.status_asm.load_4 = self.torque[3]
        
        self.status_asm.mode = self.mode
        self.status_asm.pull_down_1 = self.pull_down_sts[0]
        self.status_asm.pull_down_2 = self.pull_down_sts[1]
        self.status_asm.pull_down_3 = self.pull_down_sts[2]
        self.status_asm.pull_down_4 = self.pull_down_sts[3]
        self.status_asm.out_of_range_1 = self.out_of_range_sts[0]
        self.status_asm.out_of_range_2 = self.out_of_range_sts[1]
        self.status_asm.out_of_range_3 = self.out_of_range_sts[2]
        self.status_asm.out_of_range_4 = self.out_of_range_sts[3]
        
        self.status_asm.header.stamp = rospy.Time.now()
        self.status_asm_pub.publish(self.status_asm)



    
    def main(self):
        
        count = 0
        
        while not rospy.is_shutdown():
            
            if not self.freeze:
                if self.mode == 0: # solo simulazione
                    self.get_tf()
                    self.calculate_fi()
                    self.delta = ([0.0]*4)
                    #print("coppia",self.torque)
                elif self.mode == 1: # solo inseguitore
                    self.get_tf()
                    #if count%10 == 0:
                    self.follower()
                    self.pull_down_sts = ([False]*4)
                    self.output_fi()
                elif self.mode == 2: # solo SIL
                    self.get_tf()
                    self.calculate_fi()
                    self.delta = ([0.0]*4)
                    self.pull_down_sts = ([False]*4)
                    self.output_fi()
                elif self.mode == 3: # SIL + anti sollevamento
                    self.pull_down()
                    self.get_tf()
                    self.calculate_fi()
                    self.delta = ([0.0]*4)
                    self.output_fi()
                elif self.mode == 4: # SIL + inseguitore
                    self.get_tf()
                    self.follower()
                    #if count%40 == 0:
                    self.calculate_fi()
                    self.pull_down_sts = ([False]*4)
                    self.output_fi()
                elif self.mode == 5: # SIL + inseguitore + anti soll
                    self.pull_down()
                    self.get_tf()
                    self.follower()
                    #if count%40 == 0:
                    self.calculate_fi()
                    self.output_fi()
                
            #TODO algoritmo suddivisione carico
            
            #TODO algortimo bloccante superamento ostacoli
            
            #TODO da sincronizzare poi con i motori
            # if count%40 == 0:
                # try:
                    # Moving_Status = rospy.ServiceProxy('Moving_Status', isMoving)
                    # resp1 = Moving_Status(False)
                    # return resp1
                # except rospy.ServiceException, e:
                    # print "Service call failed: %s"%e
                    
                # time.sleep(0.500)
                
                # try:
                    # Moving_Status = rospy.ServiceProxy('Moving_Status', isMoving)
                    # resp1 = Moving_Status(True)
                    # return resp1
                # except rospy.ServiceException, e:
                    # print "Service call failed: %s"%e
            
            self.set_status()
                
            count += 1
            self.rate.sleep()



if __name__ == '__main__':
    try:
        SuspensionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

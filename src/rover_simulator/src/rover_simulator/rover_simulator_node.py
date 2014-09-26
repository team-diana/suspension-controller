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

from rover_simulator.srv import set_heights

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
from visualization_msgs.msg import Marker

import tf
from tf.transformations import euler_from_quaternion
import math

#proseguire import da controller manager e spawner

class SuspensionController:
    def __init__(self):
        print("INIT")
        rospy.init_node('rover_simulator', anonymous=True)
        rospy.on_shutdown(self.on_shutdown)
        
        self.hz = 20
        self.rate = rospy.Rate(self.hz)#self.diagnostics_rate)
#        self.ID = rospy.get_param('/dynamixel/suspension_port/connected_ids')
#        self.nID = len(self.ID)
#        self.deltaH_inertial = ([0.0]*4)
#        self.deltaH_chassis = ([0.0]*4)
#        self.angoli_chassis = ([0.0]*3)
#        self.posa_chassis = ([0.0]*3)
#        self.angoli_chassis_virtuale = ([0.0]*3)
#        self.posa_chassis_virtuale = ([0.0]*3)
#        self.altezza = 0.0
#        self.range_front = 0.4
#        self.range_post = 0.4
#        self.deltaH_chassis_virtuale = 0.0
#        self.fi = ([0.0]*4)
#        self.Z_ruote = ([0.0]*4)
#        self.deltaH_hub = ([0.0]*4)
#        self.angoli_sosp = ([0.0]*4)
#        
        self.joint_state_out = JointStateOut()
        self.joint_state_out.name = []
        self.joint_state_out.position = []
        self.joint_state_out.velocity = []
        self.joint_state_out.effort = []
        
        self.joint_state = JointState()
        self.joint_state.name = []
        self.joint_state.current_pos = 0.0
        self.joint_state.velocity = 0.0
        self.joint_state.load = 0.0

        self.arm = ([0.0]*4)
        self.arm_cmd = ([0.0]*4)
        self.req_height = ([0.0]*4)
        self.altezza = ([0.0]*4)
        self.angoli_chassis = ([0.0]*4)
        self.posa_chassis = ([0.0]*4)
        self.Z_ruote = ([0.0]*4)
        
        self.pitch = 0.0
        self.roll = 0.0
        self.pitchf = ([0.0]*21)
        self.rollf = ([0.0]*21)
        self.pointer = 0
        
        self.joint_speed = rospy.get_param('/motore_1_controller/joint_speed', 0.5/6.3)
        
        self.marker = Marker()
        
        self.start()
        
        time.sleep(2) # todo mettere metodo più furbo per aspettare messaggi validi dalla scheda adc
        
        print("START")
        
        self.main()


    def on_shutdown(self):
        self.stop
    
    
    def start(self):
        self.running = True        
        
        self.joint_state_out_pub_1 = rospy.Publisher('/motore_1_controller/joint_states', JointStateOut)
        self.joint_state_out_pub_2 = rospy.Publisher('/motore_2_controller/joint_states', JointStateOut)
        self.joint_state_out_pub_3 = rospy.Publisher('/motore_3_controller/joint_states', JointStateOut)
        self.joint_state_out_pub_4 = rospy.Publisher('/motore_4_controller/joint_states', JointStateOut)
        self.joint_state_pub_1 = rospy.Publisher('/motore_1_controller/arm/state', JointState)
        self.joint_state_pub_2 = rospy.Publisher('/motore_2_controller/arm/state', JointState)
        self.joint_state_pub_3 = rospy.Publisher('/motore_3_controller/arm/state', JointState)
        self.joint_state_pub_4 = rospy.Publisher('/motore_4_controller/arm/state', JointState)
        
        self.vis_pub = rospy.Publisher('/visualization_marker', Marker)
        
        self.command_arm1_sub = rospy.Subscriber('/motore_1_controller/arm/command', Float64,  self.process_arm_1)     
        self.command_arm2_sub = rospy.Subscriber('/motore_2_controller/arm/command', Float64,  self.process_arm_2)  
        self.command_arm3_sub = rospy.Subscriber('/motore_3_controller/arm/command', Float64,  self.process_arm_3)  
        self.command_arm4_sub = rospy.Subscriber('/motore_4_controller/arm/command', Float64,  self.process_arm_4)  
             
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.service_height = rospy.Service('rover_simulator/set_heights', set_heights, self.handle_set_height)
        
        self.marker.header.frame_id = "/base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "step"
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.0
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.vis_pub.publish( self.marker )
        
        self.marker.header.frame_id = "/base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "step"
        self.marker.id = 1
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.0
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.vis_pub.publish( self.marker )
        
        self.marker.header.frame_id = "/base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "step"
        self.marker.id = 2
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.0
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.vis_pub.publish( self.marker )
        
        self.marker.header.frame_id = "/base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "step"
        self.marker.id = 3
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.0
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.vis_pub.publish( self.marker )

    def stop(self):
        self.running = False
        
        self.joint_state_out_pub_1.unregister()
        self.joint_state_out_pub_2.unregister()
        self.joint_state_out_pub_3.unregister()
        self.joint_state_out_pub_4.unregister()
        self.joint_state_pub_1.unregister()
        self.joint_state_pub_2.unregister()
        self.joint_state_pub_3.unregister()
        self.joint_state_pub_4.unregister()    
        
        self.vis_pub.unregister()
        
        self.command_arm1_sub.unregister()
        self.command_arm2_sub.unregister()
        self.command_arm3_sub.unregister()
        self.command_arm4_sub.unregister()
        

    def handle_set_height(self, req):
        X_ruote = ([0.0]*4)
        Y_ruote = ([0.0]*4)
        self.req_height[0] = req.height1
        self.req_height[1] = req.height2
        self.req_height[2] = req.height3
        self.req_height[3] = req.height4
        
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
               X_ruote[i]= trans[0]
               Y_ruote[i]= trans[1]
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               print('no_info')
               X_ruote[i]= 0.0
               Y_ruote[i]= 0.0
               continue
        for i in range(0,4):
            self.marker.header.frame_id = "/base_link"
            self.marker.header.stamp = rospy.Time.now()
            self.marker.ns = "step"
            self.marker.id = i
            self.marker.type = Marker.CUBE
            self.marker.action = Marker.MODIFY;
            self.marker.pose.position.x = X_ruote[i];
            self.marker.pose.position.y = Y_ruote[i];
            self.marker.pose.position.z = 0;
            self.marker.scale.x = 0.1;
            self.marker.scale.y = 0.1;
            self.marker.scale.z = self.req_height[i];
            self.marker.color.a = 1.0;
            self.marker.color.r = 0.0;
            self.marker.color.g = 1.0;
            self.marker.color.b = 0.0;
            self.vis_pub.publish( self.marker );
        
        return [True]
    
    def process_arm_1(self, msg):
        self.arm_cmd[0] = msg.data
        
    def process_arm_2(self, msg):
        self.arm_cmd[1] = msg.data

    def process_arm_3(self, msg):
        self.arm_cmd[2] = msg.data
        
    def process_arm_4(self, msg):
        self.arm_cmd[3] = msg.data




    def calculate_pose(self):
        i_hmax = 0
        hmax = 0.0
        
        i_braccio_max = 0
        braccio_max = 0.0
        bracci = ([0.0]*4)
        
           
        #TODO ma questo serve?????
        try:
            (trans,rot) = self.listener.lookupTransform('inertial', 'chassis', rospy.Time(0))
            print('posizione chassis: ',trans)
            angles = euler_from_quaternion(rot)
            print('rotazione chassis: ',angles)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('no_info') #continue
            angles = [0,0,0]
            trans = [0,0,0]
        self.angoli_chassis = angles
        self.posa_chassis = trans
        
        for i in range(0,4):
            bracci[i] = 0.2 * math.sin(self.arm[i]) *math.cos(math.fabs(angles[1]))
            if bracci[i] >= braccio_max :
               braccio_max = bracci[i]
               i_braccio_max = i
               #print(i+1)
        
        for i in range(0,4):
            self.altezza[i] = 0.2 * math.cos(self.arm[i]) *math.cos(math.fabs(angles[1])) + self.req_height[i] + 0.09
            if self.altezza[i] >= hmax :
               hmax = self.altezza[i]
               i_hmax = i
               print(i+1)
        
        print('bracci: ',bracci)
        print('comandi hub: ',self.arm_cmd)
        print('angoli hub: ',self.arm)
        print('altezza hub: ',self.altezza)
        if i_hmax == 0 or i_hmax == 2:
            if i_hmax == 0:
               print(1)
               self.pitch = math.asin(( max(self.altezza[1],self.altezza[3]) - self.altezza[i_hmax] )/(0.26+bracci[i_hmax]+bracci[1]))
               self.roll = - math.asin(( self.altezza[2] - self.altezza[0] )/(pow(pow(0.45,2)+pow(bracci[0]-bracci[2],2),0.5)))
               print((pow(pow(0.45,2)+pow(bracci[0]-bracci[2],2),0.5)))
            else: #i_hmax == 2
               print(3)
               self.pitch = math.asin(( max(self.altezza[1],self.altezza[3]) - self.altezza[i_hmax] )/(0.26+bracci[i_hmax]+bracci[3]))
               self.roll = math.asin(( self.altezza[0] - self.altezza[2] )/(pow(pow(0.45,2)+pow(bracci[0]-bracci[2],2),0.5)))
               print((pow(pow(0.45,2)+pow(bracci[0]-bracci[2],2),0.5)))
        else: # i_hmax == 1 or i_hmax == 3:
            if i_hmax == 1:
               print(2)
               self.pitch = - math.asin(( max(self.altezza[0],self.altezza[2]) - self.altezza[i_hmax] )/(0.26+bracci[i_hmax]+bracci[0]))
               self.roll = - math.asin(( self.altezza[3] - self.altezza[1] )/(pow(pow(0.45,2)+pow(bracci[1]-bracci[3],2),0.5)))
               print((pow(pow(0.45,2)+pow(bracci[1]-bracci[3],2),0.5)))
            else: #i_hmax == 3
               print(4)
               self.pitch = - math.asin(( max(self.altezza[0],self.altezza[2]) - self.altezza[i_hmax] )/(0.26+bracci[i_hmax]+bracci[2]))
               self.roll = math.asin(( self.altezza[1] - self.altezza[3] )/(pow(pow(0.45,2)+pow(bracci[1]-bracci[3],2),0.5)))   
               print((pow(pow(0.45,2)+pow(bracci[1]-bracci[3],2),0.5))) 
        
           
        
        print('pitch: ',self.pitch)
        print('roll: ',self.roll)
        
        #print (self.pointer)
        self.pitchf[self.pointer] = self.pitch
        self.rollf[self.pointer] = self.roll
        if self.pointer == 19:
           self.pointer = 0
        else:
           self.pointer += 1
        self.pitchf[20] = 0.0
        self.rollf[20] = 0.0
        for i in range(0,20):
            self.pitchf[20] = self.pitchf[20]+self.pitchf[i]
            self.rollf[20] = self.rollf[20]+self.rollf[i]
        self.pitchf[20] = self.pitchf[20]/20.0
        self.rollf[20] = self.rollf[20]/20.0
        
        #test!
        self.pitchf[20] = self.pitch
        self.rollf[20] = self.roll
        
        
        print('pitch: ',self.pitchf[20])
        print('roll: ',self.rollf[20])
        print('---------------')
        
        self.br.sendTransform( ( 0.0, 0.0, 0.0), tf.transformations.quaternion_from_euler(self.rollf[20], self.pitchf[20], angles[2]) , rospy.Time.now(), 'chassis','inertial');
        
    def output_arm(self):
        speed = 0.0

        for i in range(0,4):
            if self.arm_cmd[i] > self.arm[i]:
               speed = self.joint_speed
               self.arm[i] = self.arm[i] + speed*(1/float(self.hz))
            elif self.arm_cmd[i] < self.arm[i]:
               speed = - self.joint_speed
               self.arm[i] = self.arm[i] + speed*(1/float(self.hz))
            else:
               speed = 0
               self.arm[i] = self.arm[i] + speed*(1/float(self.hz))
               
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
               self.Z_ruote[i]= trans[2]-0.090
           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               print('no_info')
               self.Z_ruote[i]= 0.0
               continue
           #print('translation: ',trans)
           #angles = euler_from_quaternion(rot)
           #print('rotation: ',[(180.0/math.pi)*i for i in angles])
            
           if i == 0:
              self.joint_state_out.name = []
              self.joint_state_out.position = []
              self.joint_state_out.velocity = []
              self.joint_state_out.effort = []
              if self.Z_ruote[i] <= self.req_height[i]+0.02:
                  self.joint_state_out.effort.append(1.0)
                  self.joint_state.load = 1.0
              else:
                  self.joint_state_out.effort.append(-1.0)
                  self.joint_state.load = -0.26
                  print('ruota sollevata', i+1)
              self.joint_state_out.name.append("hub_f_l")
              self.joint_state.name = "hub_f_l"
              self.joint_state_out.velocity.append(speed)
              self.joint_state_out.position.append(self.arm[i])
              self.joint_state_out_pub_1.publish(self.joint_state_out)
              self.joint_state.velocity = speed
              self.joint_state.current_pos = self.arm[i]
              self.joint_state_pub_1.publish(self.joint_state)
           if i == 1:
              self.joint_state_out.name = []
              self.joint_state_out.position = []
              self.joint_state_out.velocity = []
              self.joint_state_out.effort = []
              self.joint_state.current_pos = []
              self.joint_state.velocity = []
              self.joint_state.load = []
              if self.Z_ruote[i] <= self.req_height[i]+0.02:
                  self.joint_state_out.effort.append(1.0)
                  self.joint_state.load = 1.0
              else:
                  self.joint_state_out.effort.append(-1.0)
                  self.joint_state.load = -0.26
                  print('ruota sollevata', i+1)
              self.joint_state_out.name.append("hub_p_l")
              self.joint_state.name = "hub_p_l"
              self.joint_state_out.velocity.append(speed)
              self.joint_state_out.position.append(self.arm[i])
              self.joint_state_out_pub_2.publish(self.joint_state_out)
              self.joint_state.velocity = speed
              self.joint_state.current_pos = self.arm[i]
              self.joint_state_pub_2.publish(self.joint_state)
           if i == 2:
              self.joint_state_out.name = []
              self.joint_state_out.position = []
              self.joint_state_out.velocity = []
              self.joint_state_out.effort = []
              self.joint_state.current_pos = []
              self.joint_state.velocity = []
              self.joint_state.load = []
              if self.Z_ruote[i] <= self.req_height[i]+0.02:
                  self.joint_state_out.effort.append(1.0)
                  self.joint_state.load = 1.0
              else:
                  self.joint_state_out.effort.append(-1.0)
                  self.joint_state.load = -0.26
                  print('ruota sollevata', i+1)
              self.joint_state_out.name.append("hub_f_r")
              self.joint_state.name = "hub_f_r"
              self.joint_state_out.velocity.append(speed)
              self.joint_state_out.position.append(self.arm[i])
              self.joint_state_out_pub_3.publish(self.joint_state_out)
              self.joint_state.velocity = speed
              self.joint_state.current_pos = self.arm[i]
              self.joint_state_pub_3.publish(self.joint_state)
           if i == 3:
              self.joint_state_out.name = []
              self.joint_state_out.position = []
              self.joint_state_out.velocity = []
              self.joint_state_out.effort = []
              self.joint_state.current_pos = []
              self.joint_state.velocity = []
              self.joint_state.load = []
              if self.Z_ruote[i] <= self.req_height[i]+0.02:
                  self.joint_state_out.effort.append(1.0)
                  self.joint_state.load = 1.0
              else:
                  self.joint_state_out.effort.append(-1.0)
                  self.joint_state.load = -0.26
                  print('ruota sollevata', i+1)
              self.joint_state_out.name.append("hub_p_r")
              self.joint_state.name = "hub_p_r"
              self.joint_state_out.velocity.append(speed)
              self.joint_state_out.position.append(self.arm[i])
              self.joint_state_out_pub_4.publish(self.joint_state_out)
              self.joint_state.velocity = speed
              self.joint_state.current_pos = self.arm[i]
              self.joint_state_pub_4.publish(self.joint_state)
               
        print('altezza imposta: ',self.req_height)
        print('altezza ruote: ',self.Z_ruote)
        print('---------------')
        
#    def output_fi(self):     #TODO vedere se va filtrata l'uscita o se serve cambiare i parametri del PID sui motori
#        self.recovery = 0
#        for i in range(0,4): 
#            phi = self.fi[i]
#            if phi < 0.19: 
#                 phi = 0.19
#                 self.recovery += 1
#            elif phi > 0.95: 
#                 phi = 0.95
#                 self.recovery += 1
#                  
#            if i == 0:
#                self.command_arm1_pub.publish(phi)
#            elif i == 1:
#                self.command_arm2_pub.publish(phi)
#            elif i == 2:
#                self.command_arm3_pub.publish(phi)
#            elif i == 3:
#                self.command_arm4_pub.publish(phi)
#            
#        if self.recovery == 3:        
#                 print('attivata recovery altezza')
#                 print('---------------')
#                 if not self.recovery_height: #prima attivazione
#                    self.recovery_height = True
#                    self.req_height_temp = self.req_height
#                 if self.req_height >= 0.061:
#                    self.req_height = self.req_height - 0.001
#                 else: #successive
#                    if self.req_height >= 0.061:
#                        self.req_height = self.req_height - 0.001
#            
#        elif self.req_height <= self.req_height_temp and self.recovery_height: #non sono piu' in recovery e devo correggere altezza
#                self.req_height = self.req_height + 0.001
#                if self.req_height_temp == self.req_height:
#                   self.recovery_height = False
#                   print('rientro recovery altezza')
#                   print('---------------')

        
    def main(self):    
        #rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            self.calculate_pose()
            self.output_arm()
            
            self.rate.sleep()



if __name__ == '__main__':
    try:
        SuspensionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#
# Based on code by Mattia Marenco:
# Copyright (c) 2013, Mattia Marenco <mattia.marenco@teamdiana.org>
# All rights reserved.

from numpy import arccos, average, cos, fabs, pi

import rospy

import time

import tf
from tf.transformations import euler_from_quaternion

from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as JointStateOut
from std_msgs.msg import Float64

from dynamixel_controllers.srv import SetTorque
from suspension_controller.srv import Freeze, SetMode, SetHeight, StopAll

from suspension_controller.constants.config import MAX_PHI_ANGLE, MIN_PHI_ANGLE, MAX_WHEEL_ANGLE, MIN_WHEEL_ANGLE, TEST_LOCKING_DELAY, TEST_SLEEP_DELAY, TORQUE_MAX_LOAD, TORQUE_SAMPLE_SIZE

class Arm:
    def __init__(self, index):
        self.torquef = [0] * TORQUE_SAMPLE_SIZE
        self.pointer = 0
        self.torque = 0.0
        self.position = 0.0  # was self.pos_arm
        self.error = 0.0  # was self.error_arm
        self.motor_temp = 0.0
        self.delta = 0.0

        self.suspension_angle = 0.0  # was self.angoli_sosp

        self.phi = 0.0  # was self.fi

        self.pull_down_sts = False
        self.out_of_range_sts = False

        self.joint_state_out = JointStateOut()

        self.computed_height = 0.0 # was self.altezza

        # This might be needed
        self.index = index
        if self.index == 1:
            self.location = 'f_l'
        elif self.index == 2:
            self.location = 'p_l'
        elif self.index == 3:
            self.location = 'f_r'
        elif self.index == 4:
            self.location = 'p_r'
        else:
            raise RuntimeError("Invalid index of wheel (%d must be 1-4)" % self.index)

    def read_arm_data(self, msg):
        self.torquef[self.pointer] = msg.load

        if self.pointer == TORQUE_SAMPLE_SIZE - 1:
            self.pointer = 0
        else:
            self.pointer += 1

        avg = average(self.torquef)

        self.torque = avg
        self.pistion = msg.current_pos
        self.error = msg.error

        self.motor_temp = int(msg.motor_temps[self.index - 1])

    def publish(self):
        self.command_pub = rospy.Publisher('/motore_%d_controller/command' % self.index, Float64)
        self.command_arm_pub = rospy.Publisher('/motore_%d_controller/arm/command' % self.index, Float64)
        self.command_tor_pub = rospy.Publisher('/motore_%d_controller/vel_tor/command' % self.index, Float64)
        self.arm_status_sub = rospy.Subscriber('/motore_%d_controller/arm/state' % self.index, JointState, self.read_arm_data)

    def unpublish(self):
        self.command_pub.unregister()
        self.command_arm_pub.unregister()
        self.command_tor_pub.unregister()
        self.arm_status_sub.unregister()

    def stop(self):
        try:
            motor_motion = rospy.ServiceProxy('/motore_%d_controller/set_torque' % self.index, SetTorque)
            resp = motor_motion(False)
            rospy.loginfo("Suspended motor_%d" % self.index)
            return resp.response
        except rospy.ServiceException as e:
            rospy.logerror("Stop service call failed: %s" % e)
        return None

    def update_status(self, status):
        getattr(status, "pos_%d" % self.index)(self.position)

        getattr(status, "motor_pos_%d" % self.index)(self.position + self.error)

        getattr(status, "command_%d" % self.index)(self.phi)
        getattr(status, "delta_%d" % self.index)(self.phi + self.delta)

        # TODO: What is self.Z_route?!
        getattr(status, "height_%d" % self.index)(self.Z_route)

        getattr(status, "motor_temp_%d" % self.index)(self.motor_temp)

        getattr(status, "load_%d" % self.index)(self.torque)
        
        getattr(status, "pull_down_%d" % self.index)(self.pull_down_sts)

        getattr(status, "out_of_range_%d" % self.index)(self.out_of_range_sts)
        status.roll = self.chassis_angles[0]
        status.pitch = self.chassis_angles[1]
        status.current_height = self.computed_height - 0.05

    def delta_follower(self, limit):
        if self.error > limit or self.error < -limit:
            self.delta = self.error

    def read_suspension_angle(self, msg):
        self.suspension_angle = getattr(msg, "sosp%d" % self.index)

    # TODO: This is a really ugly function
    # TODO: What are ang_p and ang_n?
    def test_wheel(self, init_pos_high=True):
        if init_pos_high:
            ang_p = pi * 2.0
            ang_n = 0.0
        else:
            ang_p = 0.0
            ang_n = pi * 2.0

        # wheels 2 and 3 should be linked
        if self.index == 2 or self.index == 3:
            angle = ang_n
        else:
            angle = ang_p

        self.command_pub.publish(angle)
        rospy.loginfo("Tested moving wheel %d", self.index)

        time.sleep(TEST_SLEEP_DELAY);

        if init_pos_high:
            ang_p = 0.5
            ang_n = -0.5
        else:
            ang_p = -0.5
            ang_n = 0.5

        if self.index == 2 or self.index == 3:
            angle = ang_n
        else:
            angle = ang_p

        self.command_tor_pub.publish(angle)
        rospy.loginfo("Tested stepping of wheel %d", self.index)

        time.sleep(TEST_SLEEP_DELAY);

        if init_pos_high:
            ang_p = pi
            ang_n = -pi
        else:
            ang_p = -pi
            ang_n = pi

        if self.index == 2 or self.index == 3:
            angle = ang_n
        else:
            angle = ang_p

        rospy.loginfo("Testing locking of wheel %d", self.index)
        for _ in range(0, 7):
            self.command_pub.publish(angle)
            time.sleep(TEST_LOCKING_DELAY)

    # XXX: this is work in progress and completely broken
    # look at mattia's thesis
    def compute_transfer_function(self, range_front, range_post, req_height, joint_state_out_pub, tf_listener, tf_broadcaster):
        rospy.loginfo("Getting transfer function for arm #%d..." % self.index)

        deltaH_chassis = 0.0
        self.deltaH_inertial = 0.0
        angles = [ 0.0, 0.0, 0.0 ]
        trans = [ 0.0, 0.0, 0.0 ]

        try:
            (trans, rot) = tf_listener.lookupTransform('chassis', 'wheel_%s' % self.location, rospy.Time(0))
            deltaH_chassis = -trans[2] + 0.090
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('no_info chassis to wheel %i', self.index)

        try:
            (trans, rot) = tf_listener.lookupTransform('inertial', 'wheel_%s' % self.location, rospy.Time(0))
            self.deltaH_inertial = -trans[2] + 0.090
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info inertial to wheel %i", self.index)

        try:
            (trans, rot) = tf_listener.lookupTransform('base_link', 'wheel_%s' % self.location, rospy.Time(0))
            self.Z_ruote = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info base_link to wheel %i", self.index)
            self.Z_ruote = 0.0

        try:
            (trans, rot) = tf_listener.lookupTransform('base_link', 'chassis_virtuale', rospy.Time(0))
            rospy.logdebug("posizione chassis_virtuale: %f %f %f", trans[0], trans[1], trans[2])

            angles = euler_from_quaternion(rot)

            rospy.logdebug("angoli chassis_virtuale: %f %f %f", angles[0], angles[1], angles[2])

            self.angoli_chassis_virtuale = angles
            self.posa_chassis_virtuale = trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('no_info base_link toi chassis_virtuale')

        # XXX: WHAT DO THESE CONSTANTS MEAN?!
        tf_broadcaster.sendTransform((0.0, 0.0, self.deltaH_inertial), (0.0, 0.0, 0.0, 1.0) , rospy.Time.now(), 'inertial', 'base_link');

        try:
            (trans, rot) = tf_listener.lookupTransform('base_link', 'chassis', rospy.Time(0))
            rospy.logdebug("Chassis position: %f %f %f", trans[0], trans[1], trans[2])

            angles = euler_from_quaternion(rot)

            rospy.logdebug("Chassis orientation by angles: %f %f %f", angles[0], angles[1], angles[2])
            self.computed_height = min(0.05 + range_front * cos(fabs(angles[1])) * cos(fabs(angles[0])), 0.05 + range_post * cos(fabs(angles[1])) * cos(fabs(angles[0])), deltaH_chassis)
            rospy.logdebug("Computed height from range finder module: %f", self.computed_height - 0.05)
            rospy.logdebug("Requested height: %f", req_height - 0.05)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info base_link to chassis")
            angles = [ 0.0, 0.0, 0.0 ]
            trans = [ 0.0, 0.0, 0.0 ]

        self.chassis_angles = angles
        posa_chassis = trans

        self.deltaH_chassis_virtuale = req_height - self.computed_height
        rospy.logdebug("distanza da inertial a chassis virtuale: %f", self.deltaH_chassis_virtuale)

        joint_state_out = self.joint_state_out

        joint_state_out.name = [ "delta_h" ]
        joint_state_out.position = [ self.deltaH_chassis_virtuale ]
        joint_state_out.velocity = []
        joint_state_out.effort = []
        joint_state_out.header.stamp = rospy.Time.now()
        joint_state_out_pub.publish(joint_state_out)

        joint_state_out.name = [ "rpy_r" ]
        joint_state_out.position = [ -self.chassis_angles[0] ]
        joint_state_out.velocity = []
        joint_state_out.effort = []
        joint_state_out.header.stamp = rospy.Time.now()
        joint_state_out_pub.publish(joint_state_out)

        joint_state_out.name = [ "rpy_y" ]
        joint_state_out.position = [ -self.chassis_angles[1] ]
        joint_state_out.velocity = []
        joint_state_out.effort = []
        joint_state_out.header.stamp = rospy.Time.now()
        joint_state_out_pub.publish(joint_state_out)


    def compute_phi(self, joint_state_out_pub):
        try:
            self.phi = arccos((self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote) / 0.20)
        except ValueError:
            rospy.logwarn("out_of_range on wheel %i", self.index)
            # TODO: What is the if statement trying to do?
            if ((self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote) / 0.20) > 1.:
                self.phi = MIN_WHEEL_ANGLE
            elif ((self.posa_chassis_virtuale[2] - 0.011 - self.Z_ruote) / 0.20) < 0.:
                self.phi = MAX_WHEEL_ANGLE

        joint_state_out = JointStateOut()

        joint_state_out.name = [ "hub_%s_virtuale" % self.location ]
        joint_state_out.position = [ self.phi + self.delta ]
        joint_state_out.velocity = []
        joint_state_out.effort = []
        joint_state_out.header.stamp = rospy.Time.now()
        joint_state_out_pub.publish(joint_state_out)


    def publish_phi(self):
        if (self.pull_down_sts == False):
            phi = self.phi + self.delta
            if phi < MIN_PHI_ANGLE:
                phi = MIN_PHI_ANGLE
                self.out_of_range_sts = True
            elif phi > MAX_PHI_ANGLE:
                phi = MAX_PHI_ANGLE
                self.out_of_range_sts = True
            else:
                self.out_of_range_sts = False

            if self.torque > TORQUE_MAX_LOAD:
                rospy.logwarn("Limite coppia su %i!", self.index)
                phi = phi + 0.1

            self.command_arm_pub.publish(phi)

            if self.torque > TORQUE_MAX_LOAD:
                time.sleep(0.5)


    def pull_down(self):
        if self.torque < 0.1:
            self.pull_down_sts = True
            phi = self.position - 0.5
            if phi < MIN_PHI_ANGLE:
                phi = MIN_PHI_ANGLE
            elif phi > MAX_PHI_ANGLE:
                # XXX: Shouldn't this be MAX_PHI_ANGLE?
                phi = 1.00

            rospy.logdebug("Coppia motore %i %f -> comando posizione %f", self.index, self.torque, phi)
            self.command_arm_pub.publish(phi)

            self.joint_state_out.name = [ "hub_%s_virtuale" % self.location ]
            self.joint_state_out.position = [ phi ]
            self.joint_state_out.velocity = []
            self.joint_state_out.effort = []
            self.joint_state_out.header.stamp = rospy.Time.now()
            self.joint_state_out_pub.publish(self.joint_state_out)

        else:
            self.pull_down_sts = False

        time.sleep(0.1)
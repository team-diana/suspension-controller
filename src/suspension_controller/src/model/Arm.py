# This file is released under a 3-clause BSD license, for
# more details, please consult the license.txt file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

from numpy import average


class Arm:
    def __init__(self, index):
        self.torquef = [0] * 20
        self.pointer = 0
        self.torque = 0.0
        self.position = 0.0  # was self.pos_arm
        self.error = 0.0  # was error_arm
        self.motor_temp = 0.0
        self.delta = 0.0

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
            raise RuntimeError("Invalid index of wheel (%d must be 1, 2 ,3 or 4)" % self.index)

    def process_arm(self, msg):
        self.torquef[self.pointer] = msg.load

        if self.pointer == 19:
            self.pointer = 0
        else:
            self.pointer += 1

        avg = average(self.torquef)

        self.torque = avg
        self.pos_arm = msg.current_pos
        self.error_arm = msg.error
        
        # XXX: why is this always zero?!
        # I think it should be 'self.index'
        self.motor_temp = int(msg.motor_temps[0])

    def start(self):
        self.command_pub = rospy.Publisher('/motore_%d_controller/command' % self.index, Float64)
        self.command_arm_pub = rospy.Publisher('/motore_%d_controller/arm/command' % self.index, Float64)
        self.command_tor_pub = rospy.Publisher('/motore_%d_controller/vel_tor/command' % self.index, Float64)
        self.arm_status_sub = rospy.Subscriber('/motore_%d_controller/arm/state' % self.index, JointState, self.process_arm)

    def stop(self):
        self.command_pub.unregister()  # this was missing in the original code
        self.command_arm_pub.unregister()
        self.command_tor_pub.unregister()
        self.arm_status_sub.unregister()

    def update_status(self):
        getattr(self.status_asm, "pos_%d" % self.index)(self.position)

        getattr(self.status_asm, "mot_pos_%d" % self.index)(self.position + self.error)

        # TODO: What is self.fi, self.delta and self.Z_route?!
        getattr(self.status_asm, "command_%d" % self.index)(self.fi)
        getattr(self.status_asm, "delta_%d" % self.index)(self.fi + self.delta)
        getattr(self.status_asm, "height_%d" % self.index)(self.Z_route)

        getattr(self.status_asm, "motor_%d_temp" % self.index)(self.motor_temp)

        getattr(self.status_asm, "load_%d" % self.index)(self.torque)
        
        getattr(self.status_asm, "pull_down_%d" % self.index)(self.pull_down_sts)

        getattr(self.status_asm, "out_of_range_%d" % self.index)(self.out_of_range_sts)


    def delta_follower(self, limit):
        if self.error > limit or self.error < -limit:
            self.delta = self.error
    
    def transfer_function(self):
        rospy.loginfo("Getting transfer function...")

        try:
            (trans, rot) = self.listener.lookupTransform('chassis', 'wheel_%s' % self.location, rospy.Time(0))
            self.deltaH_chassis = -trans[2] + 0.090
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('no_info chassis to wheel %i', self.index)
            self.deltaH_chassis = 0.0

#         rospy.logdebug("distanza ruote da chassis: %f %f %f %f", self.deltaH_chassis[0], self.deltaH_chassis[1], self.deltaH_chassis[2], self.deltaH_chassis[3])
        
        try:
            (trans, rot) = self.listener.lookupTransform('inertial', 'wheel_%s'%self.location, rospy.Time(0))
            self.deltaH_inertial = -trans[2] + 0.090
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info inertial to wheel %i", self.index)
            self.deltaH_inertial = 0.0

#         rospy.logdebug("distanza ruote da inertial: %f %f %f %f", self.deltaH_inertial[0], self.deltaH_inertial[1], self.deltaH_inertial[2], self.deltaH_inertial[3])

        # XXX: WHAT DO THESE CONSTANTS MEAN?!
        self.br.sendTransform((0.0, 0.0, max(self.deltaH_inertial)), (0.0, 0.0, 0.0, 1.0) , rospy.Time.now(), 'inertial', 'base_link');
        
        try:
            (trans, rot) = self.listener.lookupTransform('base_link', 'chassis', rospy.Time(0))
            rospy.logdebug("posizione chassis: %f %f %f", trans[0], trans[1], trans[2])
            angles = euler_from_quaternion(rot)

            rospy.logdebug("rotazione chassis: %f %f %f", angles[0], angles[1], angles[2])
            self.altezza = min(0.05 + self.range_front * math.cos(math.fabs(angles[1])) * math.cos(math.fabs(angles[0])), 0.05 + self.range_post * math.cos(math.fabs(angles[1])) * math.cos(math.fabs(angles[0])), min(self.deltaH_chassis))
            rospy.logdebug("altezza chassis corretta da range: %f", self.altezza - 0.05)
            rospy.logdebug("altezza richiesta: %f", self.req_height - 0.05)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info base_link to chassis")  # continue
            angles = [0, 0, 0]
            trans = [0, 0, 0]
            self.altezza = 0
        self.angoli_chassis = angles
        self.posa_chassis = trans

        
        try:
            (trans, rot) = self.listener.lookupTransform('base_link', 'wheel_%s'%self.location, rospy.Time(0))
            self.Z_ruote = trans[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no_info base_link to wheel %i", self.index)
            self.Z_ruote = 0.0

#         rospy.logdebug("distanza ruote da base_link: %f %f %f %f", self.Z_ruote[0], self.Z_ruote[1], self.Z_ruote[2], self.Z_ruote[3])
        
        
        self.deltaH_chassis_virtuale = self.req_height - self.altezza
        rospy.logdebug("distanza da inertial a chassis virtuale: %f", self.deltaH_chassis_virtuale)
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
            (trans, rot) = self.listener.lookupTransform('base_link', 'chassis_virtuale', rospy.Time(0))
            rospy.logdebug("posizione chassis_virtuale: %f %f %f", trans[0], trans[1], trans[2])
            angles = euler_from_quaternion(rot)
            # rospy.logdebug('rotation: ',[(180.0/math.pi)*i for i in angles])
            rospy.logdebug("angoli chassis_virtuale: %f %f %f", angles[0], angles[1], angles[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('no_info base_link toi chassis_virtuale')  # continue)
            angles = [0, 0, 0]
            trans = [0, 0, 0]
        self.angoli_chassis_virtuale = angles
        self.posa_chassis_virtuale = trans

if __name__ == '__main__':
    arm1 = Arm(1)
    arm2 = Arm(2)
    arm3 = Arm(3)
    arm4 = Arm(4)

    msg = {}

    arm1.process_arm(msg)
    arm2.process_arm(msg)
    arm3.process_arm(msg)
    arm4.process_arm(msg)

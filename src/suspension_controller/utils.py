#!/usr/bin/env python

import rospy
import time

def clamp(minimum, maximum, value):
    return min(max(value, minimum), maximum)

def get_transform(listener, to, fr):
    trials = 0
    max_trials = 200
    while True:
        try:
            return listener.lookupTransform(to, fr, rospy.Time(0))
        except Exception as e: 
            rospy.logdebug("failed trial %i", trials)
            if trials >= max_trials:
                raise e
            else:
                trials += 1
                time.sleep(0.01)

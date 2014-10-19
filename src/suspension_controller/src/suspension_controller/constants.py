# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

class _const:
    def __init__(self):
        # add constants here:
        self.INIT_ADC_WAIT_PERIOD = 2
        self.MAX_PHI_ANGLE = 1.21
        self.MIN_PHI_ANGLE = 0.40
        self.MIN_WHEEL_ANGLE = 0.39
        self.MAX_WHEEL_ANGLE = 1.22
        self.TEST_LOCKING_DELAY = 0.1
        self.TEST_SLEEP_DELAY = 4.0
        self.TORQUE_MAX_LOAD = 3.0
        self.TORQUE_SAMPLE_SIZE = 20

    class ConstError(TypeError): pass
    def __setattr__(self, name, value):
        if self.__dict__.has_key(name):
            raise self.ConstError("Can't modify %s, you must edit the constants module!" % name)
        self.__dict__[name] = value

config = _const()
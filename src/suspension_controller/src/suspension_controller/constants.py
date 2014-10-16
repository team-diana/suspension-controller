# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

class _const:
    def __init__(self):
        # add constants here:
        self.MIN_WHEEL_ANGLE = 0.39
        self.MAX_WHEEL_ANGLE = 1.22
        self.TORQUE_SAMPLE_SIZE = 20

    class ConstError(TypeError): pass
    def __setattr__(self, name, value):
        if self.__dict__.has_key(name):
            raise self.ConstError("Can't modify %s, you must edit the constants module!" % name)
        self.__dict__[name] = value

config = _const()
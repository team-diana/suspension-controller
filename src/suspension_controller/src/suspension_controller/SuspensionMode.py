# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

class SuspensionMode:
    def __init__(self, mode):
        self.__mode = mode

    def set(self, mode):
        self.__mode = mode

    def __getattr__(self, name):
        return getattr(self.__mode, name)
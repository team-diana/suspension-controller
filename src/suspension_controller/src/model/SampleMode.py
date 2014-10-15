# This file is released under a 3-clause BSD license, for
# more details, please consult the license.txt file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

'''
This model does not do anything; it merely acts as an interface definition of different modes of usage.
'''

class SampleMode:
    def __init__(self):
        self.name = "some mode"
        self.delta = []
        self.pull_down_sts = []

    def change_mode(self, mode):
        self.__mode = mode
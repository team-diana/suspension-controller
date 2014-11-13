# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#
# Based on code by Mattia Marenco:
# Copyright (c) 2013, Mattia Marenco <mattia.marenco@teamdiana.org>
# All rights reserved.

class SuspensionMode:
    def __init__(self, mode):
        self.__mode = mode

    def set(self, mode):
        self.__mode = mode

    def __getattr__(self, name):
        return getattr(self.__mode, name)

class BaseMode:
    def __init__(self, node):
        self.node = node
        raise Exception("Cannot instantiate object directly, use one of its sub-classes!")

    def run(self):
        raise Exception("Function not implemented!")

class Simulation(BaseMode):
    def __init__(self):
        self.name = "Simulation"
        self.name_it = "Simulazione"
        self.index = 0

    def run(self):
        self.node.get_tf()
        self.node.calculate_phi()

class Follower(BaseMode):
    def __init__(self):
        self.name = "Follower"
        self.name_it = "Inseguitore"
        self.index = 1

    def run(self):
        self.node.get_tf()
        self.node.follower()
#         self.node.pull_down_sts = ([False] * 4)
        self.node.output_phi()

class Observer(BaseMode):
    def __init__(self):
        self.name = "Observer"
        self.name_it = "Osservatore"
        self.index = 2

    def run(self):
        self.node.get_tf()
        self.node.calculate_phi()
#         self.node.delta = ([0.0] * 4)
#         self.node.pull_down_sts = ([False] * 4)
        self.node.output_phi()

class WithAntilift(BaseMode):
    def __init__(self):
        self.name = "Observer with anti-lift control"
        self.name_it = "Osservatore con antisollevamento"
        self.index = 3

    def run(self):
        self.node.pull_down()
        self.node.get_tf()
        self.node.calculate_phi()
#         self.node.delta = ([0.0] * 4)
        self.node.output_phi()

class WithFollower(BaseMode):
    def __init__(self):
        self.name = "Observer with follower control"
        self.name_it = "Osservatore con inseguitore"
        self.index = 0

    def run(self):
        self.node.get_tf()
        self.node.follower()
        self.node.calculate_phi()
#         self.node.pull_down_sts = ([False] * 4)
        self.node.output_phi()

class WithAntiliftAndFollower(BaseMode):
    def __init__(self):
        self.name = "Observer with anti-lift and follower control"
        self.name_it = "Osservatore con antisollevamento e inseguitore"
        self.index = 0

    def run(self):
        self.node.pull_down()
        self.node.get_tf()
        self.node.follower()
        self.node.calculate_phi()
        self.node.output_phi()
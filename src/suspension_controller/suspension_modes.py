# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.
#
# Based on code by Mattia Marenco:
# Copyright (c) 2013, Mattia Marenco <mattia.marenco@teamdiana.org>
# All rights reserved.

class BaseMode:
    def __init__(self):
        raise Exception("Cannot instantiate object directly, use one of its sub-classes!")

    def do_controller_cycle(self):
        raise Exception("Function not implemented!")

class Simulation(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Simulation"

    def do_controller_cycle(self):
        self.node.update_wheels_phi()

class Follower(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Follower"

    def do_controller_cycle(self):
        self.node.follower()

class Observer(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Observer"

    def do_controller_cycle(self):
        self.node.update_wheels_phi()

class WithAntilift(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Observer with anti-lift control"

    def do_controller_cycle(self):
        self.node.pull_down()
        self.node.update_wheels_phi()

class WithFollower(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Observer with follower control"

    def do_controller_cycle(self):
        self.node.follower()
        self.node.update_wheels_phi()

class WithAntiliftAndFollower(BaseMode):
    def __init__(self, node):
        self.node = node
        self.name = "Observer with anti-lift and follower control"

    def do_controller_cycle(self):
        self.node.pull_down()
        self.node.follower()
        self.node.update_wheels_phi()

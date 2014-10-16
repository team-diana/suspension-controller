# This file is released under a 3-clause BSD license, for
# more details, please consult the LICENSE file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

class BaseMode:
    def __init__(self):
        raise Exception("Cannot instantiate object directly, use one of its sub-classes!")

    def run(self):
        raise Exception("Function not implemented!")

class Simulation(BaseMode):
    def __init__(self):
        self.name = "Simulation"
        self.name_it = "Simulazione"

        self.index = 0

    def run(self):
        pass

class Follower(BaseMode):
    def __init__(self):
        self.name = "Follower"
        self.name_it = "Inseguitore"

        self.index = 1

class Observer(BaseMode):
    def __init__(self):
        self.name = "Observer"
        self.name_it = "Osservatore"

        self.index = 2

class WithAntilift(BaseMode):
    def __init__(self):
        self.name = "Observer with anti-lift control"
        self.name_it = "Osservatore con antisollevamento"

        self.index = 3

class WithFollower(BaseMode):
    def __init__(self):
        self.name = "Observer with follower control"
        self.name_it = "Osservatore con inseguitore"

        self.index = 0

class WithAntiliftAndFollower(BaseMode):
    def __init__(self):
        self.name = "Observer with anti-lift and follower control"
        self.name_it = "Osservatore con antisollevamento e inseguitore"

        self.index = 0
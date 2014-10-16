# This file is released under a 3-clause BSD license, for
# more details, please consult the license.txt file.
#
# Copyright (c) 2014, Tamer Saadeh <tamer@tamersaadeh.com>
# All rights reserved.

class Simulation:
    def __init__(self):
        self.name = "Simulation"
        self.name_it = "Simulazione"

        self.index = 0

class Inseguitore:
    def __init__(self):
        self.name = "?"
        self.name_it = "Inseguitore"

        self.index = 1

class Osservatore:
    def __init__(self):
        self.name = "Observer"
        self.name_it = "Osservatore"

        self.index = 2

class WithAntisollevamento:
    def __init__(self):
        self.name = "?"
        self.name_it = "Osservatore + antisollevamento"

        self.index = 3

class WithInseguitore:
    def __init__(self):
        self.name = "?"
        self.name_it = "Osservatore + inseguitore"

        self.index = 0

class WithAntisollevamentoAndInseguitore:
    def __init__(self):
        self.name = "?"
        self.name_it = "Osservatore + antisollevamento + inseguitore"

        self.index = 0
#!/usr/bin/env python

import numpy as np

class CircularArray():
    def __init__(self, size):
        self.size = size
        self.array = np.zeros(size)
        self.next_value_index = 0

    def push_value(self, value):
        self.array[self.next_value_index] = value
        self.next_value_index = (self.next_value_index + 1) % self.size

    @property
    def values(self):
        return np.copy(self.array)

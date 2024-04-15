import base64
import logging
import math
from array import array

from backend_server.classes.singleton import Singleton


class MapData(metaclass=Singleton):
    def __init__(self):
        self.map = None
        self.map_image = None

    def set_map(self, map):
        self.map = map
        self.map_image = map

    def get_map(self):
        return self.map


def twos_comp_byte(val):
    bits = 8
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val  # return positive value as is

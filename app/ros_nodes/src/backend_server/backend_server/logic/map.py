import base64
import logging
import math
from array import array

from backend_server.helpers.singleton import Singleton


class MapData(metaclass=Singleton):
    def __init__(self):
        self.map = None
        self.map_image = None

    def set_map(self, map):
        self.map = map
        self.map_image = convert_data_to_b64_rtr(map)

    def get_map(self):
        return self.map


def convert_data_to_b64_rtr(grid):
    width = grid.info.width
    height = grid.info.height
    height_msb = math.floor(height / 256)
    height_lsb = height % 256
    width_msb = math.floor(width / 256)
    width_lsb = width % 256
    if width_lsb > 127:
        width_lsb = twos_comp_byte(width_lsb)
    if height_lsb > 127:
        height_lsb = twos_comp_byte(height)
    logging.debug(
        f"Map received W: {width} H: {height}, W msb: {width_msb} W lsb: {width_lsb} ; H msb: {height_msb} H lsb: {height_lsb}")
    try:
        data = array('b',
                     [0x42, 0x4D, twos_comp_byte(0xBA), twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, width_lsb, width_msb, 0x00, 0x00, height_lsb,
                      height_msb, 0x00, 0x00, 0x01, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, twos_comp_byte(0x84),
                      twos_comp_byte(0xA5), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    except Exception as err:
        logging.debug(f"Crashed while transforming map to image: {err}")

    for i in range(height):
        for j in range(width):
            point_value = grid.data[width * i + j]
            if point_value == -1:
                # format = BGR , donc voici du rose
                data.append(twos_comp_byte(175))
                data.append(twos_comp_byte(175))
                data.append(twos_comp_byte(175))
            else:
                point_color = math.floor(point_value / 100 * 255)
                point_color = 255 - point_color  # invert colors for black walls and white empty
                if point_color > 127:
                    point_color = twos_comp_byte(point_color)
                data.append(point_color)
                data.append(point_color)
                data.append(point_color)
        # end of a row, add padding because BMP must have a multiple of 4
        pad_n = 4 - ((width * 3) % 4)
        if pad_n == 4:
            pad_n = 0
        for _ in range(pad_n):
            data.append(0)

    return base64.b64encode(data).decode('utf-8')


def twos_comp_byte(val):
    bits = 8
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)  # compute negative value
    return val  # return positive value as is

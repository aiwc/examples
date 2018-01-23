# Helper Functions for the rule based algorithm example in Python

import math

def distance(x1, x2, y1, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def degree2radian(deg):
    return deg * math.pi / 180

def radian2degree(rad):
    return rad * 180 / math.pi

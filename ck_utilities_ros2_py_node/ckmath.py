import numpy as np
import math


def handleDeadband(val, deadband):
    return val if abs(val) > abs(deadband) else 0


def normalizeWithDeadband(val, deadband, min_value = 0):
    val = handleDeadband(val, deadband)
    if val != 0:
        sign = np.sign(val)
        val = ((abs(val) - deadband) / (1.0 - deadband))
        val = sign * (min_value + (val - 0) * ((1.0 - min_value) / (1.0 - 0.0)))
    return val


def lirp(val, min_x, max_x, min_y, max_y):
    return (min_y + (val - min_x) * ((max_y - min_y) / (max_x - min_x)))


def within(value1, value2, threshold) -> bool:
    return True if abs(value1 - value2) < abs(threshold) else False


def hypotenuse(x, y):
    return np.sqrt((x*x)+(y*y))


def wrapMax(x, max):
    return np.fmod(max + np.fmod(x, max), max)


def wrapMinMax(x, min, max):
    return min + wrapMax(x - min, max - min)


def normalize_to_2_pi(value):
    return wrapMinMax(value, 0, (2.0 * math.pi))


def polar_angle_rad(x, y):
    return normalize_to_2_pi(np.arctan2(y, x))

def limit(v, minVal, maxVal):
    return min(maxVal, max(minVal, v))
        
def inches_to_meters(inches):
    return inches * 0.0254

def meters_to_inches(meters):
    return meters / 0.0254

def epsilonEquals(a : float, b: float, epsilon : float = 0.000001) -> bool:
    return (a - epsilon <= b) and (a + epsilon >= b)
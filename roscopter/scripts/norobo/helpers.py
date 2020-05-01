import numpy as np
from pydrake import symbolic
import math

def rotation_matrix_factory(phi, theta, gamma, sin, cos):  # B->I frame, v = R*v_b
    return np.array([[cos(gamma)*cos(theta), cos(gamma)*sin(theta)*sin(phi) - sin(gamma)*cos(phi), cos(gamma)*sin(theta)*cos(phi) + sin(gamma)*sin(phi)],
                     [sin(gamma)*cos(theta), sin(gamma)*sin(theta)*sin(phi)+cos(gamma)*cos(phi), sin(gamma)*sin(theta)*cos(phi) - cos(gamma)*sin(phi)],
                     [-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)]])


def transformation_matrix_factory(phi, theta, sin, cos, tan):  # B->I frame,  W = T * W_b
    return np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
                     [0, cos(phi), -sin(phi)],
                     [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])


def transformation_matrix(phi, theta):
    return transformation_matrix_factory(phi, theta, np.sin, np.cos, np.tan)


def rotation_matrix(phi, theta, gamma):
    return rotation_matrix_factory(phi, theta, gamma, np.sin, np.cos)


def drake_rotation_matrix(phi, theta, gamma):
    return rotation_matrix_factory(phi, theta, gamma, symbolic.sin, symbolic.cos)


def drake_transformation_matrix(phi, theta):
    return transformation_matrix_factory(phi, theta, symbolic.sin, symbolic.cos, symbolic.tan)


def solveForGain(force):
    a = 0.000015
    b = -0.024451
    c = 9.002250 - force
    sol1 = (-b+math.sqrt(b**2-4*a*c))/(2*a)
    return (sol1-1000)/1000


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


def get_identity_matrix(values, num_values):
    if len(values) != num_values:
        raise ValueError("Wrong input to upper_identity_function")
    matrix = np.identity(num_values)
    for i in range(num_values):
        matrix[i, i] = values[i]
    return matrix


def get_array_line(element, element_pos, number_of_elements):
    string_array = []
    for i in range(number_of_elements):
        append_element = "0"
        if i == element_pos:
            append_element = str(element)
        string_array.append(append_element)
    return string_array


def convert_tuple_to_array(tuple, n_cols, n_rows):
    string_array = []
    for element in tuple:
        string_array += get_array_line(element[1], element[0], n_rows)
    return np.fromstring(",".join(string_array), sep=",").reshape(n_cols, n_rows)
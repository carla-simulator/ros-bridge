#!/usr/bin/env python

#
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
"""
Tool functions to calculate vehicle physics
"""

import math
import numpy

import carla_ros_bridge.transforms as trans


def get_vector_length_squared(carla_vector):
    """
    Calculate the squared length of a carla_vector

    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :return: squared vector length
    :rtype: float64
    """
    return carla_vector.x  * carla_vector.x + \
        carla_vector.y * carla_vector.y + \
        carla_vector.z * carla_vector.z


def get_vehicle_speed_squared(carla_vehicle):
    """
    Get the squared speed of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: squared speed of a carla vehicle [(m/s)^2]
    :rtype: float64
    """
    return get_vector_length_squared(carla_vehicle.get_velocity())


def get_vehicle_speed_abs(carla_vehicle):
    """
    Get the absolute speed of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: speed of a carla vehicle [m/s >= 0]
    :rtype: float64
    """
    speed = math.sqrt(get_vehicle_speed_squared(carla_vehicle))
    return speed


def get_vehicle_acceleration_abs(carla_vehicle):
    """
    Get the absolute acceleration of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: vehicle acceleration value [m/s^2 >=0]
    :rtype: float64
    """
    return math.sqrt(get_vector_length_squared(carla_vehicle.get_acceleration()))


def get_vehicle_driving_direction_sign(carla_vehicle):
    """
    Get the driving direction of the carla vehicle as sign

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: sign of the driving direction of a carla vehicle {-1.0; 1.0}
    :rtype: float64
    """
    vector_looking_forward = trans.carla_rotation_to_directional_numpy_vector(
        carla_vehicle.get_transform().rotation)
    velocity_vector = trans.carla_velocity_to_numpy_vector(
        carla_vehicle.get_velocity())
    dot_product = numpy.dot(vector_looking_forward, velocity_vector)
    if dot_product < 0:
        driving_direction_sign = -1.0
    else:
        driving_direction_sign = 1.0
    return driving_direction_sign


def get_vehicle_speed(carla_vehicle):
    """
    Get the signed speed of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: speed of a carla vehicle [m/s >= 0]
    :rtype: float64
    """
    speed = get_vehicle_speed_abs(carla_vehicle)
    # below a certain epsilon around zero, the sign detection is not reliable
    # @todo get the driving direction directly from CARLA
    speed_sign_detection_epsilon = 0.01
    if speed > speed_sign_detection_epsilon:
        speed *= get_vehicle_driving_direction_sign(carla_vehicle)
    return speed


def get_vehicle_mass(carla_vehicle):
    """
    Get the mass of a carla vehicle (defaults to 1500kg)

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: mass of a carla vehicle [kg]
    :rtype: float64
    """
    mass = carla_vehicle.attributes.get(
        'mass', 1500.0)

    return mass


def get_acceleration_of_gravity(dummy_carla_vehicle):
    """
    Get the acceleration of gravity for a carla vehicle
    (for the moment constant at 9.81 m/s^2)

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: acceleration of gravity [m/s^2]
    :rtype: float64
    """
    acceleration = 9.81

    return acceleration


def get_weight_force(carla_vehicle):
    """
    Get the weight of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: weight of the vehicle [N]
    :rtype: float64
    """
    weight = get_vehicle_mass(carla_vehicle) * \
        get_acceleration_of_gravity(carla_vehicle)

    return weight


def get_vehicle_max_steering_angle(carla_vehicle):
    """
    Get the maximum steering angle of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: maximum steering angle [radians]
    :rtype: float64
    """
    # 80 degrees is the default max steering angle of a car
    max_steering_angle = carla_vehicle.attributes.get(
        'max_steering_angle', math.radians(80))

    return max_steering_angle


def get_vehicle_max_speed(carla_vehicle):
    """
    Get the maximum speed of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: maximum speed [m/s]
    :rtype: float64
    """
    # 180 km/h is the default max speed of a car
    max_speed = carla_vehicle.attributes.get(
        'max_speed', 180.0 / 3.6)

    return max_speed


def get_vehicle_max_acceleration(carla_vehicle):
    """
    Get the maximum acceleration of a carla vehicle

    default: 3.0 m/s^2: 0-100 km/h in 9.2 seconds

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: maximum acceleration [m/s^2 > 0]
    :rtype: float64
    """
    max_acceleration = carla_vehicle.attributes.get(
        'max_acceleration', 3.0)

    return max_acceleration


def get_vehicle_max_deceleration(carla_vehicle):
    """
    Get the maximum deceleration of a carla vehicle

    default: 8 m/s^2

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: maximum deceleration [m/s^2 > 0]
    :rtype: float64
    """
    max_deceleration = carla_vehicle.attributes.get(
        'max_deceleration', 8.0)

    return max_deceleration


def get_aerodynamic_drag_force(carla_vehicle):
    """
    Calculate the aerodynamic drag force of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: aerodynamic drag force [N]
    :rtype: float64
    """
    # see also https://en.wikipedia.org/wiki/Automobile_drag_coefficient
    default_aerodynamic_drag_coefficient = 0.3
    default_drag_reference_area = 2.37
    drag_area = carla_vehicle.attributes.get(
        'drag_area',
        default_aerodynamic_drag_coefficient * default_drag_reference_area)
    rho_air_25 = 1.184
    speed_squared = get_vehicle_speed_squared(carla_vehicle)

    aerodynamic_drag_force = 0.5 * drag_area * rho_air_25 * speed_squared
    return aerodynamic_drag_force


def get_rolling_resistance_force(carla_vehicle):
    """
    Calculate the rolling resistance force of a carla vehicle

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: rolling resistance force [N]
    :rtype: float64
    """
    # usually somewhere between 0.007 to 0.014 for car tyres
    # and between 0.0025 to 0.005 for bycicle tyres
    # see also https://en.wikipedia.org/wiki/Rolling_resistance
    rolling_resistance_coefficient = carla_vehicle.attributes.get(
        'rolling_resistance_coefficient', 0.01)
    normal_force = get_weight_force(carla_vehicle)

    rolling_resistance_force = rolling_resistance_coefficient * normal_force

    return rolling_resistance_force


def get_engine_brake_force(dummy_carla_vehicle):
    """
    Calculate the engine brake force of a carla vehicle if the gas pedal would be layed off

    As this heavily depends on the engine, the current gear and velocity, this is not
    trivial to calculate. Maybe one can get this from within Unreal to the outside,
    to enable better vehicle control.
    For the moment we just put a constant force.

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: engine braking force [N]
    :rtype: float64
    """
    return 500.0


def get_slope_force(carla_vehicle):
    """
    Calculate the force of a carla vehicle faces when driving on a slope.

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: slope force [N, >0 uphill, <0 downhill]
    :rtype: float64
    """
    dummy_roll, pitch, dummy_yaw = trans.carla_rotation_to_RPY(
        carla_vehicle.get_transform().rotation)
    slope_force = get_acceleration_of_gravity(
        carla_vehicle) * get_vehicle_mass(carla_vehicle) * math.sin(pitch)
    return slope_force


def get_vehicle_lay_off_engine_acceleration(carla_vehicle):
    """
    Calculate the acceleration a carla vehicle faces by the engine on lay off

    This respects the following forces:
    - engine brake force

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: acceleration the vehicle [m/s^2 < 0]
    :rtype: float64
    """
    return -get_engine_brake_force(carla_vehicle) / get_vehicle_mass(carla_vehicle)


def get_vehicle_driving_impedance_acceleration(carla_vehicle, reverse):
    """
    Calculate the acceleration a carla vehicle faces by the driving impedance

    This respects the following forces:
    - rolling resistance force
    - aerodynamic drag force
    - slope force

    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :param reverse: `True' if the vehicle is driving in reverse direction
    :type reverse: boolean
    :return: acceleration the vehicle [m/s^2 <= 0 on flat surface]
    :rtype: float64
    """
    # taking the following forumlar as basis
    #
    # mass * acceleration = rolling_resitance_force + aerodynamic_drag_force
    #
    # deceleration = -(rolling_resitance_force + aerodynamic_drag_force)/mass
    #
    # future enhancements: incorporate also the expected motor braking force
    #
    rolling_resistance_force = get_rolling_resistance_force(carla_vehicle)
    aerodynamic_drag_force = get_aerodynamic_drag_force(carla_vehicle)
    slope_force = get_slope_force(carla_vehicle)
    if reverse:
        slope_force = -slope_force
    deceleration = -(rolling_resistance_force +
                     aerodynamic_drag_force +
                     slope_force) /  \
        get_vehicle_mass(carla_vehicle)

    return deceleration

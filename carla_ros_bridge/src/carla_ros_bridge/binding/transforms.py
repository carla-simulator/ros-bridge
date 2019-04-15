import math

def get_vector_length_squared(carla_vector):
    """
    Calculate the squared length of a carla_vector
    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :return: squared vector length
    :rtype: float64
    """
    return carla_vector.x * carla_vector.x + \
        carla_vector.y * carla_vector.y + \
        carla_vector.z * carla_vector.z

def get_vector_abs(vector):
    """
    Get the absolute speed of a carla vehicle
    :param carla_vehicle: the carla vehicle
    :type carla_vehicle: carla.Vehicle
    :return: speed of a carla vehicle [m/s >= 0]
    :rtype: float64
    """
    speed = math.sqrt(get_vector_length_squared(vector))
    return speed

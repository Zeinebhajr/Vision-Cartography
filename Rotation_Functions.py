import numpy as np
from scipy.spatial.transform import Rotation
def quaternion_to_rotation_matrix(quaternion):
    # Convert quaternion to rotation matrix
    rotation = Rotation.from_quat(quaternion)
    rotation_matrix = rotation.as_matrix()
    return rotation_matrix

def transform_object_coordinates(rotation_matrix, object_coordinates_camera, camera_position):
    # Rotate object coordinates
    object_coordinates_rotated = np.dot(rotation_matrix, object_coordinates_camera.T).T
    # Translate object coordinates to global coordinate system (EPSG:3857)
    object_coordinates_global = object_coordinates_rotated + camera_position
    return object_coordinates_global
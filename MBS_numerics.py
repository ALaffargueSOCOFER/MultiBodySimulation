import numpy as np


def RotationMatrix(theta_x, theta_y, theta_z):
    """
    Args:
        theta_x (float): Angle de roulis (rotation autour de l'axe X).
        theta_y (float): Angle de tangage (rotation autour de l'axe Y).
        theta_z (float): Angle de lacet (rotation autour de l'axe Z).

    Returns:
        numpy.ndarray: Matrice de rotation 3x3.
    """
    # Matrice de rotation autour de l'axe X (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x)],
        [0, np.sin(theta_x), np.cos(theta_x)]
    ])

    # Matrice de rotation autour de l'axe Y (pitch)
    R_y = np.array([
        [np.cos(theta_y), 0, np.sin(theta_y)],
        [0, 1, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y)]
    ])

    # Matrice de rotation autour de l'axe Z (yaw)
    R_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z), 0],
        [0, 0, 1]
    ])

    # Combinaison des rotations : R = R_z * R_y * R_x
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R
import roboticstoolbox as rtb
from pathlib import Path
from typing import Optional
import numpy as np
import math
import rospkg
import os

def resolve_ros_package_path(package_path):
    """
    Resolves a ROS package path (e.g., "package://my_package/path/to/file.txt")
    to an absolute filesystem path.

    Args:
        package_path (str): The ROS package path string.

    Returns:
        str: The absolute filesystem path, or None if the package is not found.
    """
    if not package_path.startswith("package://"):
        return package_path  # Not a package path, return as is

    # Extract package name and relative path within the package
    parts = package_path.split('/', 3)
    if len(parts) < 4:
        # Handle cases like "package://my_package" without a subpath
        package_name = parts[2]
        relative_path = ""
    else:
        package_name = parts[2]
        relative_path = parts[3]

    try:
        rospack = rospkg.RosPack()
        package_root_path = rospack.get_path(package_name)
        absolute_path = os.path.join(package_root_path, relative_path)
        return absolute_path, package_root_path
    except rospkg.ResourceNotFound:
        print(f"Error: ROS package '{package_name}' not found.")
        return None, None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None, None

def load_urdf(urdf_path: str, gripper: Optional[str] = None) -> rtb.Robot:
    """
    Loads a URDF file and returns a roboticstoolbox robot object.
    """
    urdf_path, package_root_path = resolve_ros_package_path(urdf_path)
    links, name, urdf_string, urdf_filepath_original = rtb.Robot.URDF_read(urdf_path, tld=package_root_path)
    if gripper is None:
        robot = rtb.Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath_original)
    else:
        gripper_idx = -1
        for i, link in enumerate(links):
            if link.name == gripper:
                gripper_idx = i
                break
        if gripper_idx == -1:
            raise ValueError(f"Gripper link '{gripper}' not found in URDF file {urdf_path}")
        robot = rtb.Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath_original, gripper_links = links[gripper_idx])
    return robot


def angle_axis_error(T: np.ndarray, Td: np.ndarray) -> np.ndarray:
    """
    Returns the error vector between T and Td in angle-axis form.

    :param T: The current pose
    :param Tep: The desired pose

    :returns e: the error vector between T and Td
    """

    e = np.empty(6)

    # The position error
    e[:3] = Td[:3, -1] - T[:3, -1]

    R = Td[:3, :3] @ T[:3, :3].T

    li = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if np.linalg.norm(li) < 1e-6:
        # If li is a zero vector (or very close to it)

        # diagonal matrix case
        if np.trace(R) > 0:
            # (1,1,1) case
            a = np.zeros((3,))
        else:
            a = np.pi / 2 * (np.diag(R) + 1)
    else:
        # non-diagonal matrix case
        ln = np.linalg.norm(li)
        a = math.atan2(ln, np.trace(R) - 1) * li / ln

    e[3:] = a

    return e


def clamp_velocity(velocity: np.ndarray, max_abs: np.ndarray) -> np.ndarray:
    if len(velocity) != len(max_abs):
        raise ValueError("velocity and max_abs must be of the same length")
    for i in range(len(velocity)):
        velocity[i] = min(max_abs[i], max(-max_abs[i], velocity[i]))
    return velocity
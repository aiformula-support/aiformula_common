import os.path as osp
import re

import xacro
from ament_index_python.packages import get_package_share_directory


def get_zed_intrinsic_param_path(serial_number: str, resolution: str) -> str:
    """Returns the zed intrinsic yaml path for the given `serial_number`

    Parameters:
    ----------
    `serial_number`: Zed Serial Number
    `resolution`

    Returns:
    ----------
    the intrinsic yaml path for given `serial_number`

    Examples:
    ----------
    >>> get_zed_intrinsic_param_path("SN48442725", "HD1080")
    /path/to/colcon_ws/install/vehicle/share/vehicle/config/zedx/camera_params/SN48442725/HD1080.yaml\
    """
    intrinsic_yaml_path = osp.join(get_package_share_directory("vehicle"),
                                   "config/zedx/intrinsic/", serial_number, resolution + ".yaml")
    if not osp.exists(intrinsic_yaml_path):
        raise FileNotFoundError(intrinsic_yaml_path)
    return intrinsic_yaml_path


def convert_xacro_to_urdf(xacro_path: str, urdf_path: str) -> None:
    """Convert xacro file to urdf file

    Parameters:
    ----------
    `xacro_path`: Source xacro file path
    `urdf_path`: Converted urdf file path

    Examples:
    ----------
    >>> convert_xacro_to_urdf("/path/to/robot.xacro", "/path/to/robot.urdf")
    """
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toprettyxml(indent="  ")
    f = open(urdf_path, "w")
    f.write(robot_desc)
    f.close()


def replace_wheel_joint_type(xacro_path: str, joint_type: str) -> None:
    """
    Replace the WHEEL_JOINT_TYPE property in a xacro file with a specified joint type.

    This function searches for the `<xacro:property>` tag with the name "WHEEL_JOINT_TYPE" 
    in the given xacro file and replaces its `value` attribute with the provided joint type.

    Parameters
    ----------
    xacro_path : str
        The file path to the target `.xacro` file.
    joint_type : str
        The new joint type to be set for the `WHEEL_JOINT_TYPE` property 
        (e.g., "fixed", "continuous", "revolute").

    Examples
    --------
    >>> replace_wheel_joint_type("/path/to/robot.xacro", "continuous")
    This will change:
        <xacro:property name="WHEEL_JOINT_TYPE" value="fixed"/>
    to:
        <xacro:property name="WHEEL_JOINT_TYPE" value="continuous"/>

    Notes
    -----
    - The function overwrites the installed file, so there is no need to worry about 
      modifying the original file.
    """
    with open(xacro_path, 'r', encoding='utf-8') as file:
        content = file.read()
    pattern = r'(<xacro:property\s+name="WHEEL_JOINT_TYPE"\s+value=")([^"]*)(".*?/>)'
    new_content = re.sub(pattern, rf'\1{joint_type}\3', content)
    with open(xacro_path, 'w', encoding='utf-8') as file:
        file.write(new_content)

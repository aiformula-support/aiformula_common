import os
import sys
from ament_index_python.packages import get_package_share_directory

def get_intrinsic_param_path(vehicle_name: str) -> str:
    """Returns the intrinsic yaml path of `vehicle_name`

    Parameters:
    ----------
    `vehicle_name`: Vehicle name

    Returns:
    ----------
    the intrinsic yaml path of `vehicle_name`

    Examples:
    ----------
    >>> get_intrinsic_param_path("ai_car1")
    /path/to/colcon_ws/install/vehicle/share/vehicle/config/ai_car1.yaml\
    """
    intrinsic_yaml_path = os.path.join(
        get_package_share_directory("vehicle"), "config", vehicle_name + ".yaml")

    if not os.path.exists(intrinsic_yaml_path):
        print(f"\033[91m[get_intrinsic_param_path] Error: The file '{intrinsic_yaml_path}' does not exist.\033[0m")
        sys.exit(1)

    return intrinsic_yaml_path

from itertools import count
from typing import List
import os.path as osp
import yaml
from ament_index_python.packages import get_package_share_directory

counter = count(0)


def get_unique_name(name: str) -> str:
    """Returns a unique name

    Parameters:
    ----------
    `name`: Base name

    Returns:
    ----------
    a string with a unique number appended to the end of `name`

    Examples:
    ----------
    >>> get_unique_name("rviz2")
    rviz2_0001
    """

    unique_id = next(counter)
    return f"{name}_{unique_id:04d}"


def get_frame_ids_and_topic_names() -> List[dict]:
    """Read `launchers/config/{frame_ids.yaml, topic_list.yaml}` and return their dictionaries.

    Returns:
    ----------
    All frame ids and topic names.

    Examples:
    ----------
    >>> FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()
    >>>         _, TOPIC_NAMES = get_frame_ids_and_topic_names()
    >>> FRAME_IDS, _           = get_frame_ids_and_topic_names()
    """

    with open(osp.join(get_package_share_directory("launchers"), "config/topic_list.yaml"), "r") as yml:
        topic_names = yaml.safe_load(yml)
    with open(osp.join(get_package_share_directory("launchers"), "config/frame_id_list.yaml"), "r") as yml:
        frame_ids = yaml.safe_load(yml)
    return frame_ids, topic_names

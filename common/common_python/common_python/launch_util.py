from itertools import count

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

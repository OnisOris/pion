from enum import Enum


class CMD(Enum):
    """
    Enum класс с командами для общения между дронами
    """

    UDP_PORT = 37020
    SET_SPEED = 1
    GOTO = 2
    TAKEOFF = 3
    LAND = 4
    ARM = 5
    DISARM = 6
    SMART_GOTO = 7
    LED = 8
    STOP = 9
    SWARM_ON = 10
    SAVE = 11

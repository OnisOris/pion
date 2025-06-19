from enum import Enum


class CMD(Enum):
    """
    Enum класс с командами для общения между дронами.

    :cvar UDP_PORT: Порт для UDP-соединения.
    :cvar SET_SPEED: Команда для установки скорости.
    :cvar GOTO: Команда для перемещения к указанной точке.
    :cvar TAKEOFF: Команда для взлета.
    :cvar LAND: Команда для посадки.
    :cvar ARM: Команда для включения двигателей.
    :cvar DISARM: Команда для выключения двигателей.
    :cvar SMART_GOTO: Команда для умного перемещения к указанной точке.
    :cvar LED: Команда для управления светодиодами.
    :cvar STOP: Команда для остановки всех действий.
    :cvar SWARM_ON: Команда для активации режима "стая".
    :cvar SAVE: Команда для сохранения текущих настроек.
    :cvar SET_GROUP: Команда для установки группы.
    :cvar SET_MOD: Команда для установки мода.
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
    SET_GROUP = 12
    SET_MOD = 13

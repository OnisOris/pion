import time
from unittest.mock import MagicMock

import numpy as np

from pion.pion import Pion


class TestPion:
    def test_position(self):
        """
        Тестирование геттера position для 3D и 2D режимов
        """
        # Тест для 3D
        position_3d = np.array([1, 2, 3, 0.1, 0.2, 0.3])
        pion_3d = Pion(
            position=position_3d,
            dimension=3,
            start_message_handler_from_init=False,
        )
        assert np.allclose(position_3d, pion_3d.position)

        # Тест для 2D
        position_2d = np.array([1, 2, 0.1, 0.2])
        pion_2d = Pion(
            position=position_2d,
            dimension=2,
            start_message_handler_from_init=False,
        )
        assert np.allclose(position_2d, pion_2d.position)

    def test_attitude(self):
        """
        Тестирование геттера attitude
        """
        attitude = np.array([0.1, 0.2, 0.3, 0.01, 0.02, 0.03])
        pion = Pion(attitude=attitude, start_message_handler_from_init=False)
        assert np.allclose(attitude, pion.attitude)

    def test_thread_flags(self):
        """
        Тестирование управления потоками отправки команд
        """
        pion = Pion(start_message_handler_from_init=False)

        pion.set_v()
        assert pion.speed_flag is True
        assert len(pion.threads) == 1

        pion.speed_flag = False
        time.sleep(0.1)
        assert pion.set_v_check_flag is False

    def test_message_processing(self):
        """
        Тестирование обработки MAVLink сообщений
        """
        pion = Pion(start_message_handler_from_init=False)
        msg = MagicMock()
        msg.get_type.return_value = "LOCAL_POSITION_NED"
        msg.x, msg.y, msg.z = 1.0, 2.0, 3.0
        msg.vx, msg.vy, msg.vz = 0.1, 0.2, 0.3

        pion._process_message(msg)
        assert np.allclose(pion.position[:3], [1.0, 2.0, 3.0])
        assert np.allclose(pion.speed, [0.1, 0.2, 0.3])

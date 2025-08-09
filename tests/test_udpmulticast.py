import unittest
import threading
import time
from queue import Queue, Empty

from swarm_server import UDPMulticastClient, UDPMulticastServer, get_numeric_id


import unittest
import threading
import time
from queue import Queue, Empty

class TestUDPMulticast(unittest.TestCase):
    MULTICAST_GROUP = "224.0.0.1"  # Локальная multicast-группа
    PORT = 37020
    UNIQUE_ID = 123456
    
    def setUp(self):
        # Включаем детальное логирование
        # logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)
        
        self.queue = Queue(maxsize=100)
        self.server = UDPMulticastServer(
            server_to_agent_queue=self.queue,
            multicast_group=self.MULTICAST_GROUP,
            port=self.PORT,
            unique_id=self.UNIQUE_ID
        )
        
        self.server_thread = threading.Thread(target=self.server.start, daemon=True)
        self.server_thread.start()
        
        # Увеличиваем время ожидания запуска сервера
        time.sleep(1.0)
        
        self.client = UDPMulticastClient(
            multicast_group=self.MULTICAST_GROUP,
            port=self.PORT,
            unique_id=self.UNIQUE_ID
        )
    
    def tearDown(self):
        self.server.running = False
        time.sleep(0.5)
        try:
            self.client.socket.close()
            self.server.socket.close()
        except:
            pass
        self.server_thread.join(timeout=2.0)
    
    # def test_basic_communication(self):
    #     """Тест базовой отправки и получения сообщения"""
    #     test_state = {
    #         "id": 12345,  # Используем числовой ID
    #         "position": [1.0, 2.0, 3.0],
    #         "attitude": [0.1, 0.2, 0.3],
    #         "t_speed": [0.5, 0.5, 0.5, 0.1],
    #         "command": 10,
    #         "target_id": "target_drone",
    #         "group_id": 1
    #     }
    #
    #     self.client.send(test_state)
    #
    #     # Увеличиваем время ожидания
    #     start_time = time.time()
    #     while time.time() - start_time < 5.0:
    #         if not self.queue.empty():
    #             break
    #         time.sleep(0.1)
    #
    #     try:
    #         received = self.queue.get(timeout=0.5)
    #         self.assertEqual(received.id, 12345 % (10**12))
    #         self.assertEqual(received.command, 10)
    #         self.assertEqual(received.target_id, "target_drone")
    #         self.assertEqual(received.group_id, 1)
    #
    #         # Проверяем данные
    #         self.assertAlmostEqual(received.data[0], float(self.UNIQUE_ID % (10**12)))
    #         self.assertAlmostEqual(received.data[1], 1.0)
    #         self.assertAlmostEqual(received.data[2], 2.0)
    #         self.assertAlmostEqual(received.data[3], 3.0)
    #         self.assertAlmostEqual(received.data[4], 0.1)
    #         self.assertAlmostEqual(received.data[5], 0.2)
    #         self.assertAlmostEqual(received.data[6], 0.3)
    #         self.assertAlmostEqual(received.data[7], 0.5)
    #         self.assertAlmostEqual(received.data[10], 0.1)
    #
    #     except Empty:
    #         self.fail("Сообщение не было получено сервером")
    #
    # def test_string_id_handling(self):
    #     """Тест обработки строковых идентификаторов"""
    #     client = UDPMulticastClient(
    #         multicast_group=self.MULTICAST_GROUP,
    #         port=self.PORT,
    #         unique_id="string_id_123"
    #     )
    #
    #     test_state = {
    #         "id": "another_string_id",
    #         "position": [1.0, 2.0, 3.0]
    #     }
    #
    #     client.send(test_state)
    #
    #     # Увеличиваем время ожидания
    #     start_time = time.time()
    #     while time.time() - start_time < 5.0:
    #         if not self.queue.empty():
    #             break
    #         time.sleep(0.1)
    #
    #     try:
    #         received = self.queue.get(timeout=0.5)
    #         # Проверяем, что ID преобразованы в числа
    #         self.assertIsInstance(received.id, int)
    #         self.assertIsInstance(received.data[0], float)
    #     except Empty:
    #         self.fail("Сообщение не было получено сервером")

if __name__ == "__main__":
    unittest.main()

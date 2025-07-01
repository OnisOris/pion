from __future__ import annotations

import logging
import random
import socket
import struct
import sys
import time
from queue import Queue
from typing import Any, Dict, Union

from .datagram import DDatagram

# Настройка логирования для диагностики
logging.basicConfig(level=logging.INFO, stream=sys.stdout)
logger = logging.getLogger(__name__)


class UDPMulticastClient:
    def __init__(
        self,
        multicast_group: str,
        port: int = 37020,
        unique_id: Union[int, str] = 0,
    ) -> None:
        # Улучшенная обработка различных типов идентификаторов
        if isinstance(unique_id, str):
            # Для строк: используем хеш
            self.numeric_id = abs(hash(unique_id)) % (10**12)
        else:
            # Для чисел: используем как есть
            self.numeric_id = (
                abs(int(unique_id)) % (10**12)
                if unique_id
                else random.randint(0, int(1e12))
            )

        self.encoder = DDatagram(id=self.numeric_id)
        self.multicast_group = multicast_group
        self.port = port
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Увеличиваем TTL для лучшей доставки
            self.socket.setsockopt(
                socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32
            )

            # Разрешаем повторное использование адреса
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            logger.info(
                f"UDP Multicast Client launched for {multicast_group}:{port}"
            )
        except Exception as error:
            logger.error(f"Multicast client initialization failure: {error}")

    def send(self, state: Dict[str, Any]) -> None:
        try:
            # Обработка различных типов ID
            id_value = state.get("id", "0")
            if isinstance(id_value, str):
                numeric_id = abs(hash(id_value)) % (10**12)
            else:
                numeric_id = abs(int(id_value)) % (10**12)

            # Обновление полей DDatagram
            self.encoder.token = state.get("token", -1)
            self.encoder.id = numeric_id
            self.encoder.source = 0
            self.encoder.command = state.get("command", 0)
            self.encoder.group_id = state.get("group_id", 0)

            if "target_id" in state:
                self.encoder.target_id = state["target_id"]

            # Гарантированное преобразование в float
            pos = [float(x) for x in state.get("position", [0.0] * 3)]
            att = [float(x) for x in state.get("attitude", [0.0] * 3)]
            t_speed = [float(x) for x in state.get("t_speed", [0.0] * 4)]

            self.encoder.data = [float(self.numeric_id)] + pos + att + t_speed

            serialized = self.encoder.export_serialized()

            # Логирование для отладки
            logger.debug(
                f"Sending message to {self.multicast_group}:{self.port}"
            )
            logger.debug(f"Message content: {self.encoder.__dict__}")

            self.socket.sendto(serialized, (self.multicast_group, self.port))
        except Exception as error:
            logger.error(f"Error sending multicast message: {error}")


class UDPMulticastServer:
    def __init__(
        self,
        server_to_agent_queue: Queue[Any],
        multicast_group: str,
        port: int = 37020,
        unique_id: Union[int, str] = None,
    ) -> None:
        # Обработка различных типов идентификаторов
        if unique_id is None:
            unique_id = random.randint(0, int(1e12))

        if isinstance(unique_id, str):
            decoder_id = abs(hash(unique_id)) % (10**12)
        else:
            decoder_id = abs(int(unique_id)) % (10**12)

        self.decoder = DDatagram(id=decoder_id)
        self.server_to_agent_queue = server_to_agent_queue
        self.multicast_group = multicast_group
        self.port = port
        self.running = True

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # Разрешаем повторное использование адреса
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Привязываемся ко всем интерфейсам
            self.socket.bind(("0.0.0.0", port))

            # Кроссплатформенное присоединение к multicast-группе
            group_bin = socket.inet_aton(multicast_group)

            # Для Linux
            mreq = group_bin + struct.pack("=I", socket.INADDR_ANY)

            # Для Windows
            # mreq = group_bin + struct.pack('@I', socket.INADDR_ANY)

            self.socket.setsockopt(
                socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq
            )

            # Неблокирующий режим
            self.socket.setblocking(False)

            logger.info(
                f"UDP Multicast Server listening on {multicast_group}:{port}"
            )
        except Exception as error:
            logger.error(f"Multicast server start failure: {error}")

    def start(self) -> None:
        while self.running:
            try:
                message, addr = self.socket.recvfrom(4096)
                is_valid, payload = self.decoder.read_serialized(message)

                if is_valid:
                    logger.info(f"Received valid message from {addr}")
                    logger.debug(f"Message content: {payload}")

                    if not self.server_to_agent_queue.full():
                        self.server_to_agent_queue.put(payload)
                    else:
                        logger.warning(
                            "Receive queue is full, discarding message"
                        )
                else:
                    logger.warning(f"Received invalid message from {addr}")

            except BlockingIOError:
                # Нет данных для чтения - нормальная ситуация
                time.sleep(0.01)
            except Exception as error:
                if self.running:  # Логируем только если не остановлен
                    logger.error(f"Multicast reception error: {error}")
                time.sleep(0.1)

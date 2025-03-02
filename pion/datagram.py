import hashlib
from .datagram_pb2 import *
import random

class DDatagram:
    def __init__(self, id: int = random.randint(0, int(1e12))):
        self.token = -1
        self.id = id         
        self.source = 0
        self.command = 0
        self.data = []
        self.target_id = "" 

    def to_proto(self) -> Datagram:
        """Создаёт protobuf-объект из текущих данных."""
        proto = Datagram(
            token=self.token,
            id=self.id,
            source=self.source,
            command=self.command,
            data=self.data,
            target_id=self.target_id  # передаем новое поле
        )
        proto.hash = self._calculate_hash(proto)  # Добавляем хеш
        return proto

    def from_proto(self, proto):
        """Заполняет объект из protobuf-структуры."""
        self.token = proto.token
        self.id = proto.id
        self.source = proto.source
        self.command = proto.command
        self.data = list(proto.data)
        self.target_id = proto.target_id

    def export_serialized(self):
        """Сериализация в бинарный формат protobuf с хешем."""
        proto = self.to_proto()
        return proto.SerializeToString()

    def read_serialized(self, serialized):
        """Десериализация и проверка хеша."""
        try:
            proto = Datagram()
            proto.ParseFromString(serialized)
            original_hash = proto.hash
            proto.hash = ""  # Убираем хеш перед проверкой
            expected_hash = self._calculate_hash(proto)
            return original_hash == expected_hash, proto
        except Exception as e:
            print(f"Ошибка при десериализации: {e}")
            return False, None

    @staticmethod
    def _calculate_hash(proto):
        """Вычисляет MD5-хеш от данных без хеша."""
        proto_copy = Datagram()
        proto_copy.CopyFrom(proto)
        proto_copy.hash = ""  # Очищаем поле хеша перед вычислением
        return hashlib.md5(proto_copy.SerializeToString()).hexdigest()


import hashlib
import random

from .datagram_pb2 import Datagram


class DDatagram:
    """
    Класс DDatagram представляет собой объект, который может быть сериализован в protobuf-формат и обратно.
    """

    def __init__(self, id: int = random.randint(0, int(1e12))):
        """
        Инициализация объекта DDatagram.

        :param id: уникальный идентификатор объекта, по умолчанию генерируется случайное число.
        """
        self.token = -1
        self.id = id
        self.source = 0
        self.command = 0
        self.data = []
        self.target_id = ""
        self.group_id: int = 0

    def to_proto(self) -> Datagram:
        """
        Создаёт protobuf-объект из текущих данных.

        :return: объект Datagram, заполненный данными из текущего объекта.
        """
        proto = Datagram(
            token=self.token,
            id=self.id,
            source=self.source,
            command=self.command,
            data=self.data,
            target_id=self.target_id,
            group_id=self.group_id,
        )
        proto.hash = self._calculate_hash(proto)
        return proto

    def from_proto(self, proto):
        """
        Заполняет объект из protobuf-структуры.

        :param proto: объект Datagram, из которого будут взяты данные.
        """
        self.token = proto.token
        self.id = proto.id
        self.source = proto.source
        self.command = proto.command
        self.data = list(proto.data)
        self.target_id = proto.target_id
        self.group_id = proto.group_id

    def export_serialized(self):
        """
        Сериализация в бинарный формат protobuf с хешем.

        :return: сериализованный в бинарный формат protobuf объект.
        """
        proto = self.to_proto()
        return proto.SerializeToString()

    def read_serialized(self, serialized):
        """
        Десериализация и проверка хеша.

        :param serialized: сериализованный в бинарный формат protobuf объект.
        :return: результат проверки хеша и десериализованный объект.
        """
        try:
            proto = Datagram()
            proto.ParseFromString(serialized)
            original_hash = proto.hash
            proto.hash = ""
            expected_hash = self._calculate_hash(proto)
            return original_hash == expected_hash, proto
        except Exception as e:
            print(f"Ошибка при десериализации: {e}")
            return False, None

    @staticmethod
    def _calculate_hash(proto):
        """
        Вычисляет MD5-хеш от данных без хеша.

        :param proto: объект Datagram, для которого вычисляется хеш.
        :return: MD5-хеш от данных объекта.
        """
        proto_copy = Datagram()
        proto_copy.CopyFrom(proto)
        proto_copy.hash = ""
        return hashlib.md5(proto_copy.SerializeToString()).hexdigest()

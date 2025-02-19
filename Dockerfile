# Используем официальный базовый образ Ubuntu 22.04
FROM ubuntu:22.04

# Отключаем интерактивный режим для apt
ENV DEBIAN_FRONTEND=noninteractive

# Обновляем списки пакетов и устанавливаем Python3, pip и git
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    git

# Копируем файл setup_all.py в контейнер
COPY setup_all.py /opt/setup_all.py

# Устанавливаем рабочую директорию
WORKDIR /opt

# Выполняем автотест во время сборки: если скрипт завершится успешно, сборка продолжится
RUN python3 setup_all.py && echo "Auto-test passed!"

# Команда по умолчанию при запуске контейнера
CMD ["echo", "Auto-test passed!"]

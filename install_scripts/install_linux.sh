#!/bin/bash
cd "$(dirname "$0")"

# Устанавливаем кодировку UTF-8
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

# Проверяем, запущен ли скрипт с правами root
if [ "$EUID" -ne 0 ]; then
  echo "Скрипт должен быть запущен с правами суперпользователя. Запустите с sudo."
  exit 1
fi

# URL файла setup_all.py
FILE_URL="https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/setup_all.py"

echo "Скачиваем setup_all.py..."
if ! curl -O "$FILE_URL"; then
  echo "Ошибка скачивания файла."
  exit 1
fi

chmod +x setup_all.py

echo "Запускаем setup_all.py..."
if ! python3 setup_all.py; then
  echo "Ошибка при выполнении setup_all.py."
  exit 1
fi

echo "Установка завершена."

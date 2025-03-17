#!/bin/bash
cd "$(dirname "$0")"
# Устанавливаем кодировку UTF-8
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

# URL файла setup_all.py
FILE_URL="https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/scripts/setup_all.py"

echo "Скачиваем setup_all.py..."
curl -O "$FILE_URL" || { echo "Ошибка скачивания файла."; exit 1; }

chmod +x setup_all.py

echo "Запускаем setup_all.py..."
python3 setup_all.py

echo "Нажмите любую клавишу для завершения..."
read -n1 -s

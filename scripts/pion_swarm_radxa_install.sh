#!/bin/bash
set -e
cd "$(dirname "$0")"

export LC_ALL=C.UTF-8
export LANG=C.UTF-8

if [ "$EUID" -ne 0 ]; then
  echo "Пожалуйста, запускайте скрипт с sudo."
  exit 1
fi

REAL_USER=$(logname)
REAL_HOME=$(eval echo "~$REAL_USER")
REAL_PATH="$REAL_HOME/.local/bin:$PATH"

echo "Пользователь: $REAL_USER"
echo "Домашняя директория: $REAL_HOME"

if sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c 'command -v uv &>/dev/null'; then
    echo "✅ uv уже установлен. Установка не требуется."
else
    echo "🔧 uv не найден. Устанавливаю..."
    sudo -u "$REAL_USER" bash -c 'curl -LsSf https://astral.sh/uv/install.sh | sh'
fi

INSTALL_DIR="$REAL_HOME/pion"
VENV_DIR="$INSTALL_DIR/.venv"

echo "Создаём директорию $INSTALL_DIR..."
sudo -u "$REAL_USER" mkdir -p "$INSTALL_DIR"

echo "Создаём виртуальное окружение..."
sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "\"$REAL_HOME/.local/bin/uv\" venv --python 3.13 --prompt pion \"$VENV_DIR\""

read -p "Установить pionsdk с PyPI (1) или с Git ветки dev (2)? [1/2]: " CHOICE
if [[ "$CHOICE" == "2" ]]; then
    if dpkg -s python3-dev &>/dev/null; then
        echo "✅ python3-dev уже установлен."
    else
        echo "🔧 Устанавливаем python3-dev..."
        apt update
        apt install -y python3-dev
    fi
    echo "Устанавливаем pionsdk с Git (ветка dev)..."
    sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install \"git+https://github.com/OnisOris/pion.git@dev\""
else
    echo "Устанавливаем pionsdk с PyPI..."
    sudo -u "$REAL_USER" env PATH="$REAL_PATH" bash -c "source \"$VENV_DIR/bin/activate\" && \"$REAL_HOME/.local/bin/uv\" pip install \"pionsdk\""
fi

echo "Создаём systemd unit файл /etc/systemd/system/pion.service..."

cat > /etc/systemd/system/pion.service << EOF
[Unit]
Description=Pion Autostart Service
After=network.target

[Service]
Type=simple
ExecStart=$VENV_DIR/bin/python -m swarm_server.scripts.server_radxa
WorkingDirectory=$INSTALL_DIR
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal
User=$REAL_USER

[Install]
WantedBy=multi-user.target
EOF

echo "Перезагружаем systemd и запускаем сервис..."
systemctl daemon-reexec
systemctl daemon-reload
systemctl enable pion.service
systemctl restart pion.service

echo "✅ Установка завершена. Сервис 'pion.service' активен под пользователем $REAL_USER."

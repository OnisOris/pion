# shellcheck disable=SC1128
#!/bin/bash

# Запуск процессов в фоновом режиме и сохранение их PID
for port in {8000..8009}; do
    start_server_uni --ip localhost --port "$port" &
    pids+=($!)  # Добавление PID процесса в массив
done

# Функция для завершения всех процессов
cleanup() {
    echo "Terminating all processes..."
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null
    done
    exit 0
}

# Установка ловушки на SIGINT (Ctrl+C)
trap cleanup SIGINT

# Ожидание завершения всех процессов
wait

start_server_sim --id 1 --initial 3 3 1 1

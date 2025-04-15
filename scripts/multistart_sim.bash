#!/bin/bash

# Размер сетки и количество дронов
GRID_MIN=-4
GRID_MAX=4
NUM_DRONES=5

# Определение количества строк и столбцов
ROWS=4
COLS=4

# Расчет шага между дронами
STEP_X=$(echo "scale=5; ($GRID_MAX - $GRID_MIN) / ($COLS - 1)" | bc)
STEP_Y=$(echo "scale=5; ($GRID_MAX - $GRID_MIN) / ($ROWS - 1)" | bc)

# Массив для хранения PID процессов
pids=()

# Запуск процессов с расчетом позиций
count=1
for ((i=0; i<ROWS; i++)); do
    for ((j=0; j<COLS; j++)); do
#        if [ $count -ge $NUM_DRONES ]; then
#            break
#        fi
        x=$(echo "$GRID_MIN + $j * $STEP_X" | bc)
        y=$(echo "$GRID_MIN + $i * $STEP_Y" | bc)
        z=1
        yaw=0
        start_server_sim --id "$count" --initial "$x" "$y" "$z" "$yaw" &
        pids+=($!)
        count=$((count + 1))
    done
done

# Функция для завершения всех процессов
cleanup() {
    echo "Завершение всех процессов..."
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null
    done
    exit 0
}

# Установка ловушки на SIGINT (Ctrl+C)
trap cleanup SIGINT

# Ожидание завершения всех процессов
wait

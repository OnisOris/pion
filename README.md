# Pion module

SDK для управления дронами pioneer base по mavlink

# Подключение
Для подключения к дрону достаточно создать экземпляр класса Pion: в конструкторе инициализировано 
подключение к дрону. 
```python
from pion import Pion
ip = "127.0.0.1"
port = 8000
drone = Pion(ip=ip, mavlink_port=port)
```

При отключении от дрона можно создать новое подключение:
```commandline
drone.mavlink_socket.close()
drone.mavlink_socket = create_connection("udpout", ip, port)
```

# Методы класса Pion
- включение двигателей
```
arm()
``` 
---
 - выключение двигателей
```
disarm()
```
---
 - взлет
```
 takeoff()
```
---
 - посадка дрона
```
land()
```
---
- перемещение дрона в системе координат арены с управлением с микроконтроллера дрона
```
goto(x, y, z, yaw) 
```
---
- перемещение дрона в системе координат арены с управлением по скорости с внешнего устройства
```
goto_from_outside(x, y, z, accuracy) 
```
---
- задание дрону скорости которое необходимо параллельно менять для 
задания других скоростей. 
```
send_speed(vx, vy, vz, yaw_rate)
```
---
- запускает отдельный поток, который будет отсылать дрону вектор скорости из t_speed

```
set_v()
``` 

---

- Запуск записи траектории и скоростей дрона во времени
```commandline
drone.attitude_write()
```
drone.trajectory - поле, содержащее в себе траекторию - матрицу 12xn, где каждая строка -
это [x, y, z, yaw, vx, vy, vz, v_yaw, v_xc, v_yc, v_zc, v_yaw_c, t], где

- x, y, z, yaw - координаты дрона,
- vx, vy, vz, v_yaw - вектор-строка скорости дрона
- v_xc, v_yc, v_zc, v_yaw_c - управляющий, задающий скорость вектор 
- t - время

В итоге матрица trajectory представляет собой вектора состояний дрона во времени attitude.

# Примеры использования

```python
from pion import Pion
import sys
import time
number_drone = sys.argv[1]
drone = Pion(ip=f"10.1.100.{number_drone}", mavlink_port=5656)
time.sleep(1)
drone.arm()
time.sleep(3)
drone.takeoff()


time.sleep(5)

drone.set_v()

drone.goto_from_outside(float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
time.sleep(10)
drone.land()

drone.stop()
```
При запуске примера в качестве аргументов идут: номер дрона, координаты x, y, z
```commandline
python test.py [номер дрона] 1 1 1
```
# Класс Apion
Apion - класс наследник Pion.
Все методы остались теми же, кроме set_v_async. Данный метод запускает асинхронно метод v_while для 
параллельной отправки векторов скоростей в дрон"
```
set_v_async()
```

```python
from pion import Apion
import sys
import asyncio

number_drone = sys.argv[1]
drone = Apion(ip=f"10.1.100.210", mavlink_port=5656)


async def main():
    drone.arm()
    drone.takeoff()
    await asyncio.sleep(5)

    # Запуск асинхронного цикла отправки скорости
    v_while_task = asyncio.create_task(drone.set_v_async())

    # Попробуем двигать дрон
    await drone.goto_from_outside(0, 0, 0, 0)

    await asyncio.sleep(10)
    drone.land()

    # Остановим все задачи
    drone.stop()
    v_while_task.cancel()  # Отменим задачу, чтобы остановить цикл


if __name__ == "__main__":
    asyncio.run(main())
```

Дополнительные примеры с инструкцией можно найти по ссылке:
https://github.com/OnisOris/Danalysis


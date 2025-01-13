# Класс Apion
Apion - класс наследник Pion.
Все методы остались теми же, кроме set_v_async. 

Данный метод запускает асинхронно метод v_while для 
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
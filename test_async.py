from pion import Apion
import sys
import asyncio

args = sys.argv

number_drone = sys.argv[1]
drone = Apion(ip=f"10.1.100.{number_drone}", mavlink_port=5656)


async def main():
    if '-c' in args:
        while True:
            print(f"Attitude: {drone.attitude}")
            await asyncio.sleep(0.1)  # Асинхронная задержка с небольшим интервалом
    else:
        print("---")
        drone.land()
        drone.arm()
        drone.takeoff()
        await asyncio.sleep(5)

        print(f"Initial t_speed: {drone.t_speed}")

        # Запуск асинхронного цикла отправки скорости
        v_while_task = asyncio.create_task(drone.set_v_async())

        # Попробуем двигать дрон
        await drone.goto_from_outside(float(args[2]), float(args[3]), float(args[4]), float(args[5]))

        print(f"After goto_from_outside t_speed: {drone.t_speed}")

        await asyncio.sleep(10)
        drone.land()

        # Остановим все задачи
        drone.stop()
        v_while_task.cancel()  # Отменим задачу, чтобы остановить цикл


if __name__ == "__main__":
    asyncio.run(main())



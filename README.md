# Pion

SDK для управления дронами [geoscan pioneer base](https://geoscan.education/pioneer-base) по mavlink.

Дроны с модификацией ультразвуковой системы навигации [локус](https://www.geoscan.ru/ru/products/pioneer/locus) и
с raspberry pi zero 2w (модификация "Арена").

# Установка

## Через pip

```shell
pip install git+https://github.com/OnisOris/pion
```

## Установка клонированием
Клонируем репозиторий:
```shell
git clone https://github.com/OnisOris/pion
```
Вам необходимо установить build-essential:
```shell
sudo apt update
sudo apt install build-essential
```

Необходимо поставить следующие пакеты:
```shell
pip install numpy matplotlib pymavlink cython 
```
Далее необходимо собрать cython модуль:
```
python setup.py build_ext --inplace
```


# Подключение
Для подключения к дрону достаточно создать экземпляр класса Pion
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

# Подробное описание с примерами

- [Pion](./discription/pion.md) - класс управления реальными дронами

- [Spion](./discription/spion.md) - класс дрона-симулятора

- [Apion](./discription/apion.md) - Класс с реализацией асинхронного управления

- [Simulator](./discription/simulator.md) - Классы симуляторов

- [Pio](./discription/pio.md) - Асбстрактные классы




Дополнительные примеры с инструкцией можно найти по ссылке:
https://github.com/OnisOris/Danalysis



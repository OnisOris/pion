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
### Ubuntu

Вам необходимо установить build-essential и sudo apt install python3.[ВАША ВЕРСИЯ]-dev, например 
3.12:
```shell
sudo apt update
sudo apt install build-essential
sudo apt install python3-dev
sudo apt install python3.12-dev
```
### Arch linux
```
sudo pacman -S python
sudo pacman -S base-devel
```
### Сборка
Необходимо поставить следующие пакеты:
```shell
pip install numpy matplotlib pymavlink cython setuptools
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


# Подробное описание с примерами

- [Pion](./description/pion.md) - класс управления реальными дронами

- [Spion](./description/spion.md) - класс дрона-симулятора

- [Apion](./description/apion.md) - Класс с реализацией асинхронного управления

- [Simulator](./description/simulator.md) - Классы симуляторов

- [Pio](./description/pio.md) - Асбстрактные классы




Дополнительные примеры с инструкцией можно найти по ссылке:
https://github.com/OnisOris/Danalysis



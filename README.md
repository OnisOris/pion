# Pion

SDK для управления дронами [geoscan pioneer base](https://geoscan.education/pioneer-base) по mavlink.

Дроны с модификацией ультразвуковой системы навигации [локус](https://www.geoscan.ru/ru/products/pioneer/locus) и
с raspberry pi zero 2w (модификация "Арена").

## Установка

```shell
pip install pionsdk
```

```shell
pip install git+https://github.com/OnisOris/pion
```


## Установка клонированием
Данный способ вам нужен, если вы хотите модифицировать модуль, для использования вам это не нужно,
устанавливайте через pip.

```shell
pip install -e .
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

- [Pion](docs/pion.md) - класс управления реальными дронами

- [Spion](docs/spion.md) - класс дрона-симулятора

- [Apion](docs/apion.md) - Класс с реализацией асинхронного управления

- [Simulator](docs/simulator.md) - Классы симуляторов

- [Pio](docs/pio.md) - Асбстрактные классы



Дополнительные примеры с инструкцией можно найти по ссылкам:

https://github.com/OnisOris/pion_examples

https://github.com/OnisOris/Danalysis

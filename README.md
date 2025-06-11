![lokky-analyze plot](https://github.com/OnisOris/pion/blob/dev/docs/img/swarm_figure.gif)

# Pion

[Documentation](https://onisoris.github.io/pion/)

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

# Рой дронов

В pionsdk реализовано роевое управление через класс SwarmCommunicator

![SchemeImg](https://github.com/OnisOris/pion/blob/dev/docs/img/swarm_scheme.png)


## Автоустанока swarmserver на radxa

```
curl -o pion_install.sh https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/scripts/pion_swarm_radxa_install.sh
chmod +x pion_install.sh
sudo ./pion_install.sh
```

После установки достаточно запустить /scripts/vserver.py и pionsrv (ниже есть ссылка) для управления роем


# Подробное описание с примерами

- [Pion](docs/pion.md) - класс управления реальными дронами

- [Spion](docs/spion.md) - класс дрона-симулятора

- [Apion](docs/apion.md) - Класс с реализацией асинхронного управления

- [Simulator](docs/simulator.md) - Классы симуляторов

- [Pio](docs/pio.md) - Асбстрактные классы


# Дополнительно ПО

- [Консоль управления роем](https://github.com/OnisOris/pionsrv)

- [Модуль алгоритмов роя lokky](https://github.com/OnisOris/lokky)

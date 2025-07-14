Pion
====
.. note::

   Мы используем `uv <https://docs.astral.sh/uv/getting-started/installation/>`_

   Вместо "python script.py" в инструкции будет использоваться "uv run script.py"



SDK для управления дронами `geoscan pioneer base <https://geoscan.education/pioneer-base>`_ по протоколу MAVLink.

Поддерживает дроны с модификацией ультразвуковой системы навигации `локус <https://www.geoscan.ru/ru/products/pioneer/locus>`_
и с Raspberry Pi Zero 2W (модификация "Арена") или на radxa zero 3w.

Тесты проводились также на pioneer mini, но могут быть ошибки в некоторых методах.

Перед работой с модулем рекомендуется загрузить рекомендуемые параметры через `Pioneer Station <https://docs.geoscan.ru/pioneer/instructions/applications/pioneer_station/pioneer_station_main.html>`_.

`Параметры автопилота <https://raw.githubusercontent.com/OnisOris/pion/refs/heads/main/scripts/parameters.txt>`_

Установка
---------

.. code-block:: shell

    pip install pionsdk

.. code-block:: shell

    pip install git+https://github.com/OnisOris/pion

Установка для разработки
------------------------
Данный способ требуется только для модификации модуля. Для использования устанавливайте через pip.

.. code-block:: shell

    pip install -e .

Подключение
-----------

Для подключения к дрону создайте экземпляр класса Pion.

Частый IP для реального дрона: 10.1.100.[номер]
Порт: 5656

В примере указан IP и порт для Геоскан симулятора:

.. code-block:: python

    from pion import Pion

    ip = "127.0.0.1"
    port = 8000
    drone = Pion(ip=ip, mavlink_port=port)

Рой дронов
----------

В pionsdk реализовано роевое управление через класс SwarmCommunicator.

.. image:: https://github.com/OnisOris/pion/blob/dev/docs/img/swarm_scheme.png
   :alt: SchemeImg

Схематичное отображение дронов и векторов управления

.. image:: https://github.com/OnisOris/pion/blob/dev/docs/img/lokky_small_vis.png
   :alt: lokky-analyze plot

Автоустановка swarmserver на Radxa
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    curl -o pion_install.sh https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/scripts/pion_swarm_radxa_install.sh
    chmod +x pion_install.sh
    sudo ./pion_install.sh

После установки достаточно запустить ``/scripts/vserver.py`` и ``pionsrv`` для управления роем.

Консоль управления роем
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`pionsrv <https://github.com/OnisOris/pionsrv>`_

После установки необходимо прописать команду

.. code-block:: shell

    start_control_server

Визуализация
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Для запуска вам нужен uv:

.. code-block:: shell

    pip install uv

Или matplotlib:

.. code-block:: shell

    pip install matplotlib

После установки запустите

2D + 3D

.. code-block:: shell

    uv run ./scripts/4xvserver.py

2D

.. code-block:: shell

    uv run ./scripts/vserver.py


Подробное описание с примерами
------------------------------

- `Pion <docs/pion.md>`_ - класс управления реальными дронами
- `Spion <docs/spion.md>`_ - класс дрона-симулятора
- `Apion <docs/apion.md>`_ - класс с реализацией асинхронного управления
- `Simulator <docs/simulator.md>`_ - классы симуляторов
- `Pio <docs/pio.md>`_ - абстрактные классы

Дополнительное ПО
-----------------

- `Консоль управления роем (pionsrv) <https://github.com/OnisOris/pionsrv>`_
- `Модуль алгоритмов роя (lokky) <https://github.com/OnisOris/lokky>`_


.. toctree::


   :maxdepth: 1


   :caption: Оглавление:





   Установка модуля Pion <user/install>


   Описание работы скриптов <examples/index>


   pion


   spion


   apion


   simulator


   pio


   controller

.. Pion documentation master file, created by
   sphinx-quickstart on Mon Mar 17 15:59:33 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.


Pion документация
==================

SDK для управления дронами `geoscan pioneer base <https://geoscan.education/pioneer-base>`_ по mavlink.

Дроны с модификацией ультразвуковой системы навигации `локус <https://www.geoscan.ru/ru/products/pioneer/locus>`_ и
с raspberry pi zero 2w (модификация "Арена").

Перед установкой, убедитесь, что у вас установлен git:


* `Git <https://git-scm.com/downloads>`_

Установка на Linux
==================

Автоматическая установка
------------------------

Перейдите в папку с проектом и выполните команду:

.. code:: bash

   sudo curl -sSL https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_linux.sh | sudo bash

После выполнения скрипта активируйте виртуальное окружение 

.. code:: bash

   source .venv/bin/activate

Ручная установка
----------------

Вам необходимо установить build-essential:

.. code:: bash

   sudo apt update
   sudo apt install build-essential

Установка пакетов для расширений на C/C++:

.. code:: bash

   sudo apt install python3-dev

Если вы используете несистемный python, то вы должны поставить пакет для вашей версии python:

.. code:: bash

   python3.[номер версии]-dev

К примеру:

.. code-block:: bash

   python3.13-dev

Через pip (на windows и linux)
------------------------------

.. code-block:: bash

   pip install git+https://github.com/OnisOris/pion

Установка на windows
====================

Автоматическая установка
------------------------

Перейдите в папку со своим проектом через терминал и выполните команду

.. code-block:: powershell

    powershell -NoProfile -ExecutionPolicy Bypass -Command "Invoke-WebRequest 'https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_windows.bat' -OutFile 'install_windows.bat'; Start-Process 'install_windows.bat' -Verb RunAs"

Данная команда установит все необходимые библиотеки для сборки модуля

Ручная установка
----------------

Установите python. Тесты проводились на версиях 3.9-3.13, но рекомендую ставить >3.13.
Важно, если у вас установлен python, проверьте, что необходимые библиотеки у вас есть, если же вы не уверены, переустановите 
python по инструкции.


* `Последняя версия python <https://www.python.org/downloads/>`_

Далее при установке поставьте галки, как на картинках, а также перейдите в Customize installation.

 
.. image:: img/python_install.jpg
    :width: 600
    :target: img/python_install.jpg
    :alt: окно установки 1


Доставьте все галки в "Advanced Options"

 
.. image:: img/python_install2.jpg
    :width: 600
    :target: img/python_install2.jpg
    :alt: окно установки 2


Здесь тоже доставьте галки

 
.. image:: img/python_install3.jpg
    :width: 600
    :target: img/python_install3.jpg
    :alt: окно установки 3


Проверьте, что у вас установлен vs build tools с нужными пакетами:


.. image:: img/windows_vsbt.jpg
   :width: 600
   :target: img/windows_vsbt.jpg
   :alt: vsbt


Создание вирутального окружения
-------------------------------

В терминале перейдите в ваш проект и выполните команду:

.. code-block:: bash

   python -m venv venv

После этого активируйте виртуальное окружение:

.. code-block:: bash

   venv\Scripts\activate.bat

Установка pion
--------------

.. code-block:: bash

   pip install git+https://github.com/OnisOris/pion

Установка клонированием
-----------------------

Данный способ вам нужен, если вы хотите модифицировать моудль, для использования вам это не нужно, 
устанавливайте через pip.

Клонируем репозиторий:

.. code-block:: shell

   git clone https://github.com/OnisOris/pion

Необходимо поставить следующие пакеты (при установленном и активированным виртуальном окружении по инструкции ниже
эти команды сработают для windows):

.. code-block:: shell

   pip install numpy matplotlib pymavlink cython setuptools rich protobuf paramiko

Далее необходимо собрать cython модуль:

.. code-block::

   python setup.py build_ext --inplace

Установка на mac os
===================

Автоматическая установка
------------------------

Перейдите в папку с проектом и выполните команду:

.. code-block:: bash

   curl -sSL https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/install_scripts/install_macos.sh | bash

После выполнения скрипта активируйте виртуальное окружение 

.. code-block::

   source ./.venv/bin/activate

Подключение
===========

Для подключения к дрону достаточно создать экземпляр класса Pion

.. code-block:: python

   from pion import Pion

   ip = "127.0.0.1"
   port = 8000
   drone = Pion(ip=ip, mavlink_port=port)

Подробное описание с примерами
==============================

Дополнительные примеры с инструкцией можно найти по ссылке:
https://github.com/OnisOris/Danalysis

.. toctree::
   pion
   spion
   apion
   simulator
   pio
   :maxdepth: 2
   :caption: Оглавление:


Установка на Linux
==================

Установка через pip
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

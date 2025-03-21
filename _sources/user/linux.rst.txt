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


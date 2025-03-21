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


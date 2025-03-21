Скрипт Fisrt_flight
===================

Скрипт с примером первого полета. Взлет и посадка

**Разбор скрипта.**
1. Импортируем необходимые библиотеки и определяем их назначение:

.. code-block:: python

   from pion import Pion
   import time

2. Создание подключения к дрону по IP адресу:

.. code-block:: python

   def main():
       drone = Pion(ip="10.1.100.217")
       time.sleep(0.5)

3. Запуск двигателей:

.. code-block:: python

       drone.arm()
       time.sleep(0.5)

4. Взлет на высоту по умолчанию:

.. code-block:: python

       drone.takeoff()
       time.sleep(10)

5. Выполнение приземления:

.. code-block:: python

       drone.land()
       time.sleep(7)

6. Отключение двигателей:

.. code-block:: python

       drone.disarm()
       drone.stop()


7. Далее используем конструкцию ``if __name__ == „__main__“:``, которая является точкой входа в программу. 
   Всё, что идёт до этого условия, выполнятся всегда: и при вызове в качестве модуля и при вызове, как исполняемый файл.

.. code-block:: python

   if __name__ == "__main__":
       main()

.. note::

   Пример добавления примечаний

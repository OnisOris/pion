Скрипты симуляции роя
===================

.. image:: docs/img/simswarm.gif
  :width: 400
  :alt: swarm sim gif


Данные скрипты находятся по пути ./scripts/examples/swarmsim_example.py и ./scripts/4xSimWriter.py.

./scripts/examples/swarmsim_example.py используется для симуляции поведения большого количества единиц роя, не используя при этом
сеть. Моделируется упрощенная модель из pion.sumulator.Point_yaw.

./scripts/4xSimWriter.py используется для отображения



**Команда.**
1. Запускаем скрипт записи симуляции:

.. code-block:: shell

   uv run ./scripts/examples/swarmsim_example.py

2. Скрипт чтения симуляции:

.. code-block:: shell

   uv run ./scripts/4xSimWriter.py --decimate 5 --dt 0.1 --speed 10 --bound 100 --file ./scripted_swarm_data.npz --marker-size 10

Скрипт camera
===================

Скрипт `сamera.py <github.com/OnisOris/pion/blob/dev/scripts/examples/camera.py>`_ с примером обработки видеопотока с камеры дрона


**Код main()**

.. code-block:: python

    main():
        parser = argparse.ArgumentParser(description="Run camera reader")
        parser.add_argument("--ip", type=str, help="IP", default="10.1.100.160")
        parser.add_argument("--port", type=int, help="порт", default="8554")
        parser.add_argument(
            "--headless",
            action="store_true",
            help="Без окна (сбрасывать кадры в /tmp/last_frame.jpg)",
        )
        args = parser.parse_args()

        show = (not args.headless) and _has_display()

        cam = RTSPCamera(rtsp_url=f"rtsp://{args.ip}:{args.port}/pioneer_stream")
        run_loop(cam, show_window=show)


Для запуска видео с дрона с номером устройства 160 используем команду:

.. code-block:: bash

    uv run ./scripts/examples/camera.py --ip 10.1.100.160 --port 8554
#!/usr/bin/env python3
import sys
import os
import platform
import subprocess
import shutil


def is_admin():
    """Проверка наличия прав администратора"""
    if platform.system() == "Windows":
        try:
            import ctypes
            return ctypes.windll.shell32.IsUserAnAdmin()
        except Exception:
            return False
    else:
        return os.getuid() == 0


def install_ubuntu_dependencies():
    """Устанавливаем системные зависимости для Ubuntu"""
    print("Ubuntu: Обновляем списки пакетов и устанавливаем build-essential и python3-dev...")
    try:
        subprocess.run(["apt", "update"], check=True)
        subprocess.run(["apt", "install", "-y", "build-essential", "python3-dev"], check=True)
        print("Системные зависимости успешно установлены.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки зависимостей на Ubuntu:", e)
        sys.exit(1)


def install_windows_dependencies():
    """Проверяем и устанавливаем Visual Studio Build Tools для Windows через Chocolatey"""
    print("Windows: Проверяем наличие Visual Studio Build Tools...")
    cl_path = shutil.which("cl.exe")
    if cl_path:
        print("Найден компилятор cl.exe:", cl_path)
    else:
        print("Visual Studio Build Tools не найдены.")
        choco_path = shutil.which("choco")
        if not choco_path:
            print("Chocolatey не установлен. Пожалуйста, установите Visual Studio Build Tools вручную:")
            print("https://visualstudio.microsoft.com/visual-cpp-build-tools/")
            sys.exit(1)
        else:
            print("Chocolatey найден. Пытаемся установить Visual Studio Build Tools...")
            try:
                # Устанавливаем Visual Studio 2022 Build Tools. Этот процесс может занять некоторое время.
                subprocess.run(["choco", "install", "visualstudio2022buildtools", "-y"], check=True)
                print("Visual Studio Build Tools установлены.")
            except subprocess.CalledProcessError as e:
                print("Ошибка установки Visual Studio Build Tools через Chocolatey:", e)
                sys.exit(1)


def install_python_package():
    """Устанавливаем пакет pion через pip"""
    print("Устанавливаем пакет pion через pip...")
    try:
        subprocess.run(["pip", "install", "git+https://github.com/OnisOris/pion"], check=True)
        print("Пакет pion успешно установлен.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки пакета pion:", e)
        sys.exit(1)


def main():
    current_os = platform.system()
    print("Определена операционная система:", current_os)

    if not is_admin():
        print("Ошибка: Скрипт должен быть запущен с правами администратора.")
        sys.exit(1)

    if current_os == "Linux":
        # Предполагается, что это Ubuntu или совместимый дистрибутив (используем apt)
        install_ubuntu_dependencies()
    elif current_os == "Windows":
        install_windows_dependencies()
    else:
        print("Операционная система", current_os, "не поддерживается данным скриптом.")
        sys.exit(1)

    install_python_package()


if __name__ == "__main__":
    main()

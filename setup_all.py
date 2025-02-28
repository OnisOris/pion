#!/usr/bin/env python3
import sys
import os
import platform
import subprocess
import shutil
import io

# Обеспечиваем вывод в UTF-8
if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.detach(), encoding='utf-8')

def is_admin():
    """Проверка наличия прав администратора"""
    if platform.system() == "Windows":
        try:
            import ctypes
            return ctypes.windll.shell32.IsUserAnAdmin()
        except Exception:
            return False
    elif platform.system() == "Darwin":
        return True
    else:
        return os.getuid() == 0

def install_ubuntu_dependencies():
    """Устанавливаем системные зависимости для Ubuntu"""
    print("Ubuntu: Обновляем списки пакетов и устанавливаем build-essential и python3-dev...")
    try:
        subprocess.run(["apt", "update"], check=True)
        subprocess.run(["apt", "install", "-y", "build-essential", "python3-dev", "git"], check=True)
        print("Системные зависимости успешно установлены.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки зависимостей на Ubuntu:", e)
        sys.exit(1)

def install_arch_dependencies():
    """Устанавливаем системные зависимости для Arch Linux"""
    print("Arch Linux: Обновляем пакеты и устанавливаем base-devel и git...")
    try:
        subprocess.run(["pacman", "-Syu", "--noconfirm", "base-devel", "git"], check=True)
        print("Системные зависимости для Arch Linux успешно установлены.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки зависимостей на Arch Linux:", e)
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
                result = subprocess.run(
                    ["choco", "install", "visualstudio2022buildtools", "-y"],
                    check=False
                )
                if result.returncode in (0, 3010):
                    if result.returncode == 3010:
                        print("Visual Studio Build Tools установлены, но требуется перезагрузка.")
                    else:
                        print("Visual Studio Build Tools установлены.")
                else:
                    print("Ошибка установки Visual Studio Build Tools через Chocolatey: Return code", result.returncode)
                    sys.exit(1)
            except subprocess.CalledProcessError as e:
                print("Ошибка установки Visual Studio Build Tools через Chocolatey:", e)
                sys.exit(1)

def install_macos_dependencies():
    """Устанавливаем системные зависимости для macOS"""
    print("macOS: Проверяем наличие Homebrew и устанавливаем необходимые зависимости...")
    brew_path = shutil.which("brew")
    if not brew_path:
        print("Homebrew не найден. Пожалуйста, установите Homebrew с https://brew.sh/ и запустите скрипт снова.")
        sys.exit(1)
    try:
        subprocess.run(["brew", "update"], check=True)
        # Проверка наличия Xcode Command Line Tools
        if subprocess.run(["xcode-select", "-p"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
            print("Xcode Command Line Tools не найдены. Устанавливаем их...")
            subprocess.run(["xcode-select", "--install"], check=True)
        print("Устанавливаем необходимые пакеты через Homebrew: python3 и git...")
        subprocess.run(["brew", "install", "python3", "git"], check=True)
        print("Системные зависимости для macOS успешно установлены.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки зависимостей на macOS:", e)
        sys.exit(1)

def install_python_package():
    """Создаем виртуальное окружение (.venv) в папке проекта и устанавливаем пакет pion через pip в него"""
    venv_dir = ".venv"
    python_executable = shutil.which("python") or shutil.which("python3")

    if not python_executable:
        print("Ошибка: Python не найден. Пожалуйста, установите Python.")
        sys.exit(1)

    if not os.path.exists(venv_dir):
        print("Создаем виртуальное окружение в папке проекта...")
        try:
            subprocess.run([python_executable, "-m", "venv", venv_dir], check=True)
        except subprocess.CalledProcessError as e:
            print("Ошибка создания виртуального окружения:", e)
            sys.exit(1)
    else:
        print("Виртуальное окружение уже существует.")

    # Определяем путь к pip в виртуальном окружении
    if platform.system() == "Windows":
        pip_executable = os.path.join(venv_dir, "Scripts", "pip.exe")
    else:
        pip_executable = os.path.join(venv_dir, "bin", "pip")

    print("Устанавливаем пакет pion через pip в виртуальном окружении...")
    try:
        subprocess.run([pip_executable, "install", "git+https://github.com/OnisOris/pion"], check=True)
        print("Пакет pion успешно установлен в виртуальном окружении.")
    except subprocess.CalledProcessError as e:
        print("Ошибка установки пакета pion в виртуальном окружении:", e)
        sys.exit(1)

def main():
    current_os = platform.system()
    print("Определена операционная система:", current_os)

    if not is_admin():
        print("Ошибка: Скрипт должен быть запущен с правами администратора.")
        sys.exit(1)

    if current_os == "Linux":
        if os.path.exists("/etc/arch-release"):
            install_arch_dependencies()
        else:
            install_ubuntu_dependencies()
    elif current_os == "Windows":
        install_windows_dependencies()
    elif current_os == "Darwin":
        install_macos_dependencies()
    else:
        print("Операционная система", current_os, "не поддерживается данным скриптом.")
        sys.exit(1)

    install_python_package()

if __name__ == "__main__":
    main()

@echo off
REM Устанавливаем кодовую страницу UTF-8 для корректного отображения кириллицы
chcp 65001 >nul

REM Проверяем, запущен ли скрипт с правами администратора
net session >nul 2>&1
if %errorLevel%==0 (
    echo Скрипт запущен с правами администратора.
) else (
    echo Ошибка: скрипт должен быть запущен с правами администратора.
    pause
    exit /b 1
)

REM Проверяем наличие Chocolatey (для установки зависимостей Windows)
where choco >nul 2>&1
if %errorLevel% neq 0 (
    echo Chocolatey не найден. Устанавливаем Chocolatey...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"

    timeout /t 5 >nul
    where choco >nul 2>&1
    if %errorLevel% neq 0 (
        echo Ошибка установки Chocolatey.
        pause
        exit /b 1
    ) else (
        echo Chocolatey успешно установлен.
    )
) else (
    echo Chocolatey уже установлен.
)

REM URL файла setup_all.py
set "FILE_URL=https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/setup_all.py"

echo Скачиваем setup_all.py...
curl -O %FILE_URL%
if errorlevel 1 (
    echo Ошибка скачивания файла.
    pause
    exit /b 1
)

echo Запускаем setup_all.py...
python setup_all.py

pause

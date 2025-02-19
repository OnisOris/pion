@echo off
cd /d %~dp0
REM Set the code page to UTF-8 for proper character display
chcp 65001 >nul

REM Check if Python is installed in the system
where python >nul 2>&1
if %errorLevel% neq 0 (
    echo Error: Python is not installed or not added to PATH.
    echo Please install Python and add it to PATH, then run the script again.
    pause
    exit /b 1
)

REM Check if the script is running with administrator privileges
net session >nul 2>&1
if %errorLevel%==0 (
    echo Script is running with administrator privileges.
) else (
    echo Error: The script must be run as administrator.
    pause
    exit /b 1
)

REM Check for Chocolatey (to install Windows dependencies)
where choco >nul 2>&1
if %errorLevel% neq 0 (
    echo Chocolatey not found. Installing Chocolatey...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"

    timeout /t 5 >nul
    where choco >nul 2>&1
    if %errorLevel% neq 0 (
        echo Error installing Chocolatey.
        pause
        exit /b 1
    ) else (
        echo Chocolatey installed successfully.
    )
) else (
    echo Chocolatey is already installed.
)

REM Set the URL for the setup_all.py file
set "FILE_URL=https://raw.githubusercontent.com/OnisOris/pion/refs/heads/dev/setup_all.py"

echo Downloading setup_all.py...
curl -O %FILE_URL%
if errorlevel 1 (
    echo Error downloading the file.
    pause
    exit /b 1
)

echo Running setup_all.py...
python setup_all.py

pause

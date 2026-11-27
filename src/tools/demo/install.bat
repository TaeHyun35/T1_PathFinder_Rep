@echo off
chcp 65001 >nul
rem 처음 한 번: 파이썬 3.11 이 깔린 노트북에서 필요한 것 설치
cd /d %~dp0
python -m venv venv
call venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
echo.
echo 설치 끝. 이제 setup_ssh.bat 로 로봇 연결을 한 번 설정하세요.
pause

@echo off
chcp 65001 > nul
cd /d "%~dp0"
echo 손(hand.py)과 VR 서버(vr\server.py)에 필요한 것만 설치합니다. 1분 정도 걸립니다.
python -m venv .venv || (echo Python이 없습니다. python.org에서 3.11을 설치해 주세요. 설치할 때 Add python.exe to PATH 체크 & pause & exit /b)
.venv\Scripts\python -m pip install pyserial cryptography
echo.
echo 설치 끝.
echo   손 조종:   .venv\Scripts\python hand\hand.py
echo   VR 서버:   .venv\Scripts\python vr\server.py
pause

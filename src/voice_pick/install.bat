@echo off
chcp 65001 > nul
cd /d "%~dp0"
echo [1/3] 가상환경 만들기
python -m venv .venv || (echo Python이 없습니다. python.org에서 3.11을 설치해 주세요. & pause & exit /b)
echo [2/3] torch 설치
nvidia-smi > /dev/null 2>&1 && (.venv\Scripts\python -m pip install torch --index-url https://download.pytorch.org/whl/cu128) || (.venv\Scripts\python -m pip install torch)
echo [3/3] 나머지 설치
.venv\Scripts\python -m pip install -r requirements.txt
echo 설치 끝. run.bat를 더블클릭하세요.
pause

@echo off
chcp 65001 >nul
cd /d %~dp0
if exist venv\Scripts\python.exe (set PY=venv\Scripts\python.exe) else (set PY=python)
set PYTHONIOENCODING=utf-8
start "" "https://taehyun35.github.io/T1_PathFinder_Rep/demo/"
%PY% -u demo.py --no-browser
pause

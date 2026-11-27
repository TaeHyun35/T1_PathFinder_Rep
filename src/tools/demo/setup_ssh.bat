@echo off
chcp 65001 >nul
rem 로봇(파이)에 비밀번호 없이 접속되게 (처음 한 번). 핫스팟을 켜고 로봇을 켠 뒤 실행
if not exist "%USERPROFILE%\.ssh\id_ed25519" ssh-keygen -t ed25519 -N "" -f "%USERPROFILE%\.ssh\id_ed25519"
echo 로봇 비밀번호를 한 번 입력하세요
type "%USERPROFILE%\.ssh\id_ed25519.pub" | ssh raspberry@192.168.137.4 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
ssh -o BatchMode=yes raspberry@192.168.137.4 "echo 연결 성공"
pause

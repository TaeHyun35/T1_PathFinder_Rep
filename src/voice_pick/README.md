# 말로 시키면 찾아서 잡고 내려놓기 (테스트 25·26)

오병주 테스트 25(3곳 집기)와 26(전체 연결)을 누구나 다시 해 볼 수 있게 정리한 것입니다.

"공 잡아"라고 말하면 다음 순서로 움직입니다.
1. 컴퓨터가 말을 받아씁니다 (Whisper).
2. 웹캠 화면에서 공을 찾습니다 (YOLO-World).
3. 공이 화면 왼쪽, 가운데, 오른쪽 중 어디에 있는지 봅니다.
4. 그 방향으로 로봇 손목을 돌립니다 (왼쪽 150, 가운데 90, 오른쪽 30).
5. 사람이 공을 손바닥에 대 주면 꽉 쥐고, 3초 들고 있다가 손을 펴서 내려놓습니다.

공이 안 보이면 주먹을 쥐고, 아무 말도 안 하면 움직이지 않습니다.

> 손은 팔이 없어서 물건 쪽으로 다가가지 못합니다. 그래서 물건은 사람이 손바닥에 대 줘야 합니다 (테스트 25 결과).

## 받는 방법

**ZIP으로 받기 (Git 몰라도 됨)**
1. 저장소 첫 화면(https://github.com/TaeHyun35/T1_PathFinder_Rep) 위쪽 초록색 **Code** 버튼 → **Download ZIP**
2. 받은 ZIP을 한글이 없는 경로에 풀기 (예: `C:obot\`)
3. 풀린 폴더 안의 `srcoice_pick` 폴더로 가서 아래 순서대로 하기

**git으로 받기 (최신판을 계속 받고 싶을 때)**
```
git clone https://github.com/TaeHyun35/T1_PathFinder_Rep.git
cd T1_PathFinder_Rep\srcoice_pick
```
나중에 최신판으로 바꾸려면 `git pull`

## 들어 있는 파일

| 파일 | 설명 |
| --- | --- |
| `voice_pick.py` | 실행 프로그램 |
| `requirements.txt` | 설치할 파이썬 패키지 목록 |
| `microbit-serial-range.hex` | micro:bit에 올릴 프로그램 (PC에서 각도를 받아 손을 움직임) |
| `install.bat` / `run.bat` | 윈도우에서 더블클릭으로 설치하고 실행 |

## 준비물

- AiHand 로봇 손 (micro:bit 꽂힌 상태, 전원 켜기)
- micro:bit와 PC를 잇는 USB 선
- 웹캠 (책상 위 물건 자리를 비추게 놓기)
- 마이크 (웹캠 마이크도 되지만 USB 마이크가 더 잘 알아들음)
- 물건: 빨간 공, 흰 코 스프레이 (지금까지 잡히는 것으로 확인된 두 개)
- 윈도우 PC. NVIDIA 그래픽카드가 있으면 빠르고, 없어도 느리지만 동작합니다.

## 1. micro:bit에 프로그램 올리기 (처음 한 번)

1. micro:bit를 USB로 PC에 꽂으면 `MICROBIT`라는 USB 드라이브가 보입니다.
2. `microbit-serial-range.hex` 파일을 그 드라이브에 끌어다 놓습니다.
3. micro:bit 뒷면 노란 불이 깜빡이다 멈추면 끝입니다.

이 프로그램은 PC에서 `엄지,검지,중지,약지,새끼,손목,이동시간` 한 줄을 받아 손을 움직이고 `OK`라고 답합니다. 예: `50,130,130,130,130,90,400`은 손을 펴고 손목을 정면으로 돌립니다.

## 2. 설치 (처음 한 번)

1. Python 3.11을 설치합니다 (https://www.python.org/downloads/ , 설치할 때 "Add python.exe to PATH" 체크).
2. Git을 설치합니다 (https://git-scm.com/download/win). YOLO-World가 쓰는 CLIP 패키지를 받을 때 필요합니다.
3. 이 폴더의 `install.bat`를 더블클릭합니다. 이 폴더 안에 `.venv` 가상환경을 만들고 필요한 것을 설치합니다. 10분 정도 걸릴 수 있습니다.
   - NVIDIA 그래픽카드가 있으면 그래픽카드용 torch를 먼저 설치합니다. 없으면 일반 torch가 설치됩니다.

직접 명령으로 설치하려면:

```
python -m venv .venv
.venv\Scripts\python -m pip install torch --index-url https://download.pytorch.org/whl/cu128   (그래픽카드 있을 때만)
.venv\Scripts\python -m pip install -r requirements.txt
```

YOLO 모델(약 25MB)과 Whisper 모델(약 140MB)은 처음 실행할 때 자동으로 내려받습니다.

## 3. 먼저 손목 방향 확인

```
.venv\Scripts\python voice_pick.py --look
```

손목이 30, 90, 150으로 5초씩 돕니다. 손바닥이 30에서 오른쪽, 90에서 정면, 150에서 왼쪽을 향하면 맞습니다. 반대로 향하면 `voice_pick.py` 위쪽 `ZONES`의 150과 30을 서로 바꿉니다.

## 4. 실행

`run.bat`를 더블클릭하거나:

```
.venv\Scripts\python voice_pick.py
```

화면에 웹캠이 뜨고 흰 세로선 두 줄로 왼쪽, 가운데, 오른쪽이 나뉩니다. 화면 글자를 따라 하면 됩니다.

| 화면 글자 | 할 일 |
| --- | --- |
| 물건을 칸 안에 놓으세요 (6초) | 공이나 스프레이를 한 칸에 놓기 |
| 지금 말하세요! (3.5초) | "공 잡아" 또는 "스프레이 집어" |
| '공' 가운데 칸 -> 손목 90 | 손목이 돎. 물건을 손바닥 아래쪽에 대 주기 |
| 쥐는 중... 다 쥐면 손을 떼세요 | 손이 쥐면 사람 손 떼기 |
| 쥐고 있기 (3초) | 떨어지지 않는지 보기 |
| 내려놓기 | 손이 펴짐. 물건 받기 |
| '스프레이' 안 보임 -> 주먹 | 물건을 못 찾았을 때 |

기본 4회차입니다. 중간에 멈추려면 화면을 누르고 `q` 또는 `ESC`를 누릅니다.

### 자주 쓰는 옵션

```
voice_pick.py --rounds 6        6회차
voice_pick.py --no-robot        로봇 없이 웹캠과 음성만 시험 (보낼 명령을 화면에 보여 줌)
voice_pick.py --list-mics       마이크 목록 보기
voice_pick.py --mic USB         이름에 USB가 들어간 마이크 쓰기
voice_pick.py --port COM4       micro:bit 포트 직접 정하기 (보통은 자동으로 찾음)
voice_pick.py --cam 1           웹캠 번호 바꾸기
```

## 5. 결과 확인

`captures` 폴더에 회차마다 두 가지가 저장됩니다.
- `voice_pick_날짜.avi`: 화면 녹화 (칸, 인식된 물건 상자, 알아들은 말 포함)
- `voice_pick_날짜.json`: 회차별 기록 (알아들은 말, 찾은 칸, 손목 각도, 결과)

테스트 26에서는 말이 끝나고 손목이 움직이기까지 약 0.55초였습니다 (그래픽카드 RTX 3060 기준).

## 문제 해결

| 증상 | 해결 |
| --- | --- |
| "micro:bit 포트를 못 찾았습니다" | USB를 다시 꽂기. 장치 관리자에서 COM 번호를 보고 `--port COM4` |
| 손이 안 움직임 | 로봇 전원 켜기. micro:bit에 hex를 다시 올리기 |
| "웹캠을 못 찾았습니다" | 줌, 카메라 앱 끄기. `--cam 1` |
| 말을 못 알아들음 | `--list-mics`로 마이크 확인 후 `--mic`. 마이크에 가까이, 또박또박 |
| "소리 작음"이 계속 나옴 | 마이크 볼륨 올리기, 또는 `voice_pick.py`의 `MIN_LOUD`를 0.01로 낮추기 |
| 물건이 있는데 "안 보임" | 물건을 세워 두기 (눕히면 못 찾을 수 있음). 조명 밝게 |
| 너무 느림 | 그래픽카드가 없으면 CPU로 돌아서 느림. `WHISPER_MODEL`을 "tiny"로 |

## 바꿔 볼 수 있는 것

`voice_pick.py` 맨 위 설정 부분만 바꾸면 됩니다.
- `OBJECTS`: 새 물건 추가. 예: `"컵": "cup"`. 오른쪽은 YOLO가 찾을 영어 이름이고, `HINT`에도 "컵 잡아"를 넣어 주세요.
- `FIST_TIGHT`: 꽉 쥐는 손가락 각도
- `ZONES`: 칸별 손목 각도

## 참고

- 노션: 오병주 · 테스트·실험 기록 > AI 모델·부품 실행 테스트 (2026-11-07)의 테스트 25, 26
- 알려진 한계: 펜과 접는 부채는 잡히지 않았습니다(손가락 위에 얹힘). 물건은 사람이 대 줘야 합니다.

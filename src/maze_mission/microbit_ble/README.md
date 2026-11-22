# AiHand 블루투스 (micro:bit)

손을 USB 선 없이 블루투스로 조종하는 micro:bit 프로그램과 도구.
서보를 움직이는 방법은 팀 USB 프로그램(`src/field_test/hand/microbit-serial-range.hex`)과 같다:
micro:bit → Hiwonder 서보 보드, P12(TX)/P8(RX) 115200, 프레임 `55 55 len 03 6 tL tH [id pL pH]x6`, 펄스 = 500 + 각도×2000/180.

| 파일 | 내용 |
|---|---|
| `main.ts` | micro:bit 프로그램 (MakeCode JavaScript). 블루투스 UART 로 명령을 받음 |
| `pxt.json` | MakeCode 프로젝트 설정 (bluetooth 확장, **페어링 없이 연결**) |
| `aihand-ble-v2.hex` | 빌드한 결과 (micro:bit V2 전용, 블루투스로 올릴 때 이것) |
| `aihand-ble-universal.hex` | 빌드한 결과 (V1/V2 공용, USB 로 MICROBIT 드라이브에 끌어다 놓을 때) |
| `ble_flash.py` | **USB 없이 블루투스로** 프로그램을 올리는 도구 (파이에서 실행) |
| `../ble_hand.py` | 파이에서 손을 조종하는 코드 (`hand.MicrobitHand` 와 같은 사용법) |

## 명령 (블루투스 UART, 한 줄씩)

블루투스 UART 는 한 번에 20바이트만 보내므로 명령을 짧게 나눴다.

| 명령 | 뜻 | 응답 |
|---|---|---|
| `s<i>,<각도>` | 서보 i (1 엄지, 2 검지, 3 중지, 4 약지, 5 새끼, 6 손목) 목표 각도 0~180. 아직 안 움직임 | 없음 |
| `m<ms>` | 모든 서보를 목표 각도로 ms 동안 이동 (0~1000) | `OK` |
| `o<ms>` | 손 펴기 (엄지 50, 나머지 130, 손목 90) | `OK` |
| `?` | 현재 목표 각도 | `T 50,130,130,130,130,90` |

## 올리는 법

**A. 블루투스로 (USB 없이, 파이에서)**
```
~/handenv/bin/python ble_flash.py aihand-ble-v2.hex E1:EF:DD:AE:52:EF
```
- 지금 micro:bit 에 있는 펌웨어(CODAL)와 hex 의 펌웨어 hash 가 같아야 한다 (프로그램 부분만 바꾸는 방식). 다르면 USB 로 한 번 올려야 함.
- 2026-11-22 실제로 올림: hash `4a2f0065895ea2ba` 일치, 10KB 를 7초, 재전송 0회.
- 중간에 끊겨도 micro:bit 은 블루투스 대기 상태로 켜져서 다시 실행하면 된다.

**B. USB 로**: `aihand-ble-universal.hex` 를 MICROBIT 드라이브에 끌어다 놓기.

**고쳐서 다시 빌드할 때** (PC, Node.js 필요)
```
npm install pxt pxt-microbit
npx pxt target microbit
npx pxt install
npx pxt build        # built/mbcodal-binary.hex, built/binary.hex
```
또는 makecode.microbit.org 에서 JavaScript 탭에 `main.ts` 붙여넣기 → bluetooth 확장 추가 → 프로젝트 설정 "No Pairing Required".

## 파이 준비

```
python3 -m venv --system-site-packages ~/handenv
~/handenv/bin/pip install bleak
```
`config.json` 에 `"hand": {"ble": true, "ble_addr": "<micro:bit 주소>"}` 를 넣으면 `mission.py` 가 블루투스 손을 쓴다
(주소를 비우면 이름이 "BBC micro:bit" 로 시작하는 기기를 찾음). 미션도 `~/handenv/bin/python mission.py ...` 로 실행.

손만 시험:
```
~/handenv/bin/python ble_hand.py open
~/handenv/bin/python ble_hand.py press 2      # 검지 한 번 (펴기 -> 검지만 굽히기 -> 펴기)
~/handenv/bin/python ble_hand.py code 421     # F4 F2 F1
~/handenv/bin/python ble_hand.py sweep 2      # 5도씩 굽혀 키 눌리는 각도 찾기 (매번 손을 폈다가)
```

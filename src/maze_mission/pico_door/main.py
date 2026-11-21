# 피코 WH 출구 문 (MicroPython). Thonny 로 이 파일을 피코에 main.py 로 저장하면 전원 켤 때 자동 실행.
#
# 배선
#   서보 SG90: 갈색(GND) -> GND,  빨강(+5V) -> VBUS(40번 핀, USB 전원),  주황(신호) -> GP15(20번 핀)
#   (선택) 초음파 HC-SR04: VCC -> VBUS, GND -> GND, Trig -> GP14,
#          Echo -> 1kΩ -> GP13, 그리고 GP13 -> 2kΩ -> GND   (Echo 5V 를 3.3V 로 낮추는 저항 2개, 꼭!)
#
# 주소 (같은 와이파이에서 브라우저로 시험 가능)
#   http://<피코IP>/open   문 열기
#   http://<피코IP>/close  문 닫기
#   http://<피코IP>/status 상태 + 초음파 거리
import network
import socket
import time
from machine import Pin, PWM, time_pulse_us

WIFI_NAME = "와이파이이름"
WIFI_PASS = "비밀번호"
OPEN_ANGLE = 90        # 문 열린 각도 (현장에서 조정)
CLOSE_ANGLE = 0        # 문 닫힌 각도

servo = PWM(Pin(15))
servo.freq(50)
trig, echo = Pin(14, Pin.OUT), Pin(13, Pin.IN)
led = Pin("LED", Pin.OUT)
state = "closed"


def angle(a):
    # SG90: 0.5ms(0도) ~ 2.5ms(180도), 주기 20ms
    us = 500 + (2000 * a) // 180
    servo.duty_u16(int(us * 65535 // 20000))


def distance_cm():
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    t = time_pulse_us(echo, 1, 30000)
    return -1 if t < 0 else round(t / 58.0, 1)


def connect():
    w = network.WLAN(network.STA_IF)
    w.active(True)
    w.connect(WIFI_NAME, WIFI_PASS)
    for _ in range(40):
        if w.isconnected():
            break
        led.toggle()
        time.sleep(0.5)
    if not w.isconnected():
        raise RuntimeError("와이파이 연결 실패: 이름/비밀번호 확인")
    led.value(1)
    ip = w.ifconfig()[0]
    print("피코 IP:", ip)
    return ip


angle(CLOSE_ANGLE)
connect()
s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 80))
s.listen(2)
while True:
    c, _ = s.accept()
    try:
        req = c.recv(512).decode()
        path = req.split(" ")[1] if " " in req else "/"
        if path.startswith("/open"):
            angle(OPEN_ANGLE)
            state = "open"
        elif path.startswith("/close"):
            angle(CLOSE_ANGLE)
            state = "closed"
        body = '{"state":"%s","dist_cm":%s}' % (state, distance_cm())
        c.send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n" + body)
    except Exception as e:
        print("요청 오류:", e)
    finally:
        c.close()

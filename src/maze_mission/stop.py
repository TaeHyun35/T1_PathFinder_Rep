"""비상 정지: 바퀴를 바로 멈추고, 돌고 있는 주행 프로그램도 끝나게 한다.
파이에서:  python3 ~/maze_home/stop.py
"""
import sys

open("/tmp/robot_stop", "w").close()        # 주행 프로그램이 다음 명령 때 보고 스스로 끝남
sys.path.insert(0, "/home/raspberry")
from Raspbot_Lib import Raspbot             # noqa: E402

b = Raspbot()
for i in range(4):
    b.Ctrl_Muto(i, 0)
print("STOPPED")

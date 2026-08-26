#!/usr/bin/env python3
"""키 입력만 확인하는 최소 도구 — ROS도 로봇도 건드리지 않는다.

demo_rook_run.py 와 **똑같은 방식**(cbreak + select + read(1))으로 읽는다.
여기서 키가 보이면 터미널은 정상이고 문제는 도구 쪽이다.
여기서도 안 보이면 터미널/세션 문제다.

    python3 keycheck.py      (q 로 종료)
"""
import select, sys, termios, time, tty

fd = sys.stdin.fileno()
if not sys.stdin.isatty():
    print("stdin이 tty가 아닙니다 — docker exec에 -t 가 빠졌습니다")
    raise SystemExit(1)

old = termios.tcgetattr(fd)
tty.setcbreak(fd)
termios.tcflush(fd, termios.TCIFLUSH)
print("아무 키나 누르세요. q = 종료. (10초간 입력이 없으면 안내를 찍습니다)")
last = time.time()
try:
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            ch = sys.stdin.read(1)
            print(f"  받음: {ch!r}  (0x{ord(ch):02x})", flush=True)
            last = time.time()
            if ch.lower() == "q":
                break
        else:
            time.sleep(0.02)
            if time.time() - last > 10:
                print("  ... 10초간 입력 없음", flush=True)
                last = time.time()
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print("종료")

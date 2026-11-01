# [TEST] 모터 각도 등 값 측정·동작 테스트용 파일 (대회 본 실행 파일 아님)
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import UARTDevice
from umath import pi
 
WHEEL_DIAMETER = 85
AXLE_TRACK = 120
 
STRAIGHT_SPEED = 400
STRAIGHT_ACCEL = (400, 250)
TURN_RATE = 150
TURN_ACCEL = (450, 200)
 
SETTLE_TOL = 0.5
SETTLE_GAIN = 6
SETTLE_MIN_RATE = 20
SETTLE_MAX_RATE = 60
SETTLE_TIMEOUT = 800
 
YELLOW = 1
RED = 2
GREEN = 3
 
GATE_SPEED = 500
GATE_TOL = 5
A_CLOSED = -30
A_OPEN = 65
E_RED = -15
E_YELLOW = 45

DUMP_SPEED = 300
D_CLOSED = -10
D_YELLOW = 40
D_RED = 80
 
HUSKY_PORT = Port.C
HUSKY_POWER_PIN = 1
HUSKY_ID_TO_COLOR = {1: YELLOW, 2: RED, 3: GREEN}
DETECT_MS = 300
DETECT_TIMEOUT = 1500
HUSKY_OFF_MS = 2000
HUSKY_BOOT_MS = 2500
DEAD_RATE = 27
 
hub = PrimeHub()
left = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B)
robot = DriveBase(left, right, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
robot.settings(
    straight_speed=STRAIGHT_SPEED,
    straight_acceleration=STRAIGHT_ACCEL,
    turn_rate=TURN_RATE,
    turn_acceleration=TURN_ACCEL,
)
robot.use_gyro(True)
 
gate_a = Motor(Port.A)
gate_e = Motor(Port.E)
dump = Motor(Port.D)

husky = UARTDevice(HUSKY_PORT, baudrate=115200, timeout=200, power_pin=0)
wait(HUSKY_OFF_MS)
husky = UARTDevice(HUSKY_PORT, baudrate=115200, timeout=200, power_pin=HUSKY_POWER_PIN)
husky_clock = StopWatch()
husky_buf = bytearray()
 
target_heading = 0
 
 
def start_gyro():
    global target_heading
    robot.reset(0, 0)
    target_heading = 0
    hub.light.on(Color.GREEN)
    if not hub.imu.ready():
        print("gyro not calibrated yet, heading may drift slowly")
 
 
def heading():
    return robot.angle()
 
 
def delay(ms):
    wait(ms)
 
 
def straight(distance, speed=None, then=Stop.HOLD):
    robot.settings(straight_speed=speed if speed else STRAIGHT_SPEED)
    robot.straight(distance, then=then)
 
 
def settle(target, radius=0):
    watch = StopWatch()
    first_sign = 0
    min_rate = SETTLE_MIN_RATE
    last_err = None
    last_check = 0
    moved = False
    while watch.time() < SETTLE_TIMEOUT:
        e = target - heading()
        if abs(e) <= SETTLE_TOL:
            break
        sign = 1 if e > 0 else -1
        if first_sign == 0:
            first_sign = sign
        elif sign != first_sign:
            break
        now = watch.time()
        if now - last_check >= 100:
            if last_err is not None and abs(last_err) - abs(e) < 0.2:
                min_rate = min(SETTLE_MAX_RATE, min_rate + 10)
            last_err = e
            last_check = now
        rate = sign * min(SETTLE_MAX_RATE, max(min_rate, abs(e) * SETTLE_GAIN))
        robot.drive(rate * radius * pi / 180, rate)
        moved = True
        wait(5)
    if moved:
        robot.stop()
        robot.hold()
 
 
def spin(angle, speed=None, then=Stop.HOLD):
    global target_heading
    robot.settings(turn_rate=speed if speed else TURN_RATE)
    target_heading += angle
    robot.turn(target_heading - heading(), then=then)
    if then == Stop.HOLD:
        settle(target_heading)
 
 
def pivot(angle, speed=None, forward=True, then=Stop.HOLD):
    global target_heading
    if angle == 0:
        return
    rate = speed if speed else TURN_RATE
    half = AXLE_TRACK / 2
    need = int(rate * pi / 180 * half * 1.2) + 1
    robot.settings(turn_rate=rate, straight_speed=max(STRAIGHT_SPEED, need))
    target_heading += angle
    delta = target_heading - heading()
    side = 1 if delta > 0 else -1
    if forward:
        radius = side * half
        robot.arc(radius, angle=abs(delta), then=then)
    else:
        radius = -side * half
        robot.arc(radius, angle=-abs(delta), then=then)
    if then == Stop.HOLD:
        settle(target_heading, radius)
 
 
def report(label):
    wait(300)
    h = heading()
    print(label, "| heading", round(h, 1), "| target", target_heading,
          "| error", round(h - target_heading, 1), "| distance", robot.distance())
 
 
def eat(c, wait=True):
    if c == GREEN:
        gate_a.run_target(GATE_SPEED, A_OPEN, then=Stop.HOLD, wait=wait)
    elif c == RED or c == YELLOW:
        if abs(gate_a.angle() - A_CLOSED) > GATE_TOL:
            gate_a.run_target(GATE_SPEED, A_CLOSED, then=Stop.HOLD)
        gate_e.run_target(GATE_SPEED, E_RED if c == RED else E_YELLOW, then=Stop.HOLD, wait=wait)
    else:
        print("eat: unknown color", c)
 
 
def spit_yellow(wait=True):
    dump.run_target(DUMP_SPEED, D_YELLOW, then=Stop.HOLD, wait=wait)


def spit_red(wait=True):
    dump.run_target(DUMP_SPEED, D_RED, then=Stop.HOLD, wait=wait)


def spit_close(wait=True):
    dump.run_target(DUMP_SPEED, D_CLOSED, then=Stop.HOLD, wait=wait)


def husky_read():
    global husky_buf
    husky_buf += husky.read_all()
    msgs = []
    buf = husky_buf
    i = 0
    n = len(buf)
    while i < n:
        h = buf[i]
        t = h & 0xC0
        if t == 0:
            if h == 0x04:
                msgs.append(("ACK", None))
            i += 1
            continue
        size = 1 << ((h >> 3) & 7)
        if size > 32:
            i += 1
            continue
        end = i + 1 + (1 if t == 0x80 else 0) + size
        if end >= n:
            break
        chk = 0xFF
        for k in range(i, end):
            chk ^= buf[k]
        if chk != buf[end]:
            i += 1
            continue
        if t == 0x80:
            msgs.append(("INFO", None))
        elif t == 0xC0 and size == 8:
            msgs.append(("DATA", bytes(buf[i + 1:end])))
        i = end + 1
    husky_buf = buf[i:] if n - i < 64 else bytearray()
    return msgs


def husky_restart():
    global husky_buf
    husky.clear()
    husky_buf = bytearray()


def husky_stream(ms=400):
    husky_restart()
    watch = StopWatch()
    last = -1000
    while watch.time() < ms:
        if watch.time() - last >= 100:
            husky.write(b"\x02")
            last = watch.time()
        for kind, _ in husky_read():
            if kind == "DATA":
                return True
        wait(2)
    return False


def husky_handshake():
    husky_restart()
    husky.write(bytes([0x52, 0x00, 0xC2, 0x01, 0x00, 0x6E]))
    seen_info = False
    watch = StopWatch()
    while watch.time() < 1000:
        for kind, _ in husky_read():
            if kind == "INFO":
                seen_info = True
            elif kind == "ACK" and seen_info:
                husky.write(b"\x04")
                wait(10)
                return True
        wait(2)
    return False


def husky_power_cycle():
    global husky
    print("camera link dead, power cycling camera")
    husky = UARTDevice(HUSKY_PORT, baudrate=115200, timeout=200, power_pin=0)
    wait(HUSKY_OFF_MS)
    husky = UARTDevice(HUSKY_PORT, baudrate=115200, timeout=200, power_pin=HUSKY_POWER_PIN)
    husky_clock.reset()
    husky_restart()


def detect_color(ms=DETECT_MS, timeout=DETECT_TIMEOUT):
    for attempt in range(2):
        if not (husky_stream() or (husky_handshake() and husky_stream())):
            print("detect_color: camera module not answering")
            return 0
        counts = {}
        first = None
        last = -1000
        frames = 0
        alive = False
        since = None
        limit = timeout + max(0, HUSKY_BOOT_MS - husky_clock.time())
        watch = StopWatch()
        while watch.time() < limit:
            now = watch.time()
            if now - last >= 100:
                husky.write(b"\x02")
                last = now
            for kind, p in husky_read():
                if kind != "DATA":
                    continue
                v = []
                for k in range(0, 8, 2):
                    x = p[k] | (p[k + 1] << 8)
                    v.append(x - 65536 if x >= 32768 else x)
                if any(v):
                    alive = True
                if husky_clock.time() >= HUSKY_BOOT_MS:
                    if since is None:
                        since = now
                    frames += 1
                if v[0] in HUSKY_ID_TO_COLOR and 0 <= v[1] <= 320 and 0 <= v[2] <= 240:
                    c = HUSKY_ID_TO_COLOR[v[0]]
                    counts[c] = counts.get(c, 0) + 1
                    if first is None:
                        first = now
            if first is not None and now - first >= ms:
                break
            wait(5)
        span = watch.time() - since if since is not None else 0
        if counts or alive or attempt or span < 500 or frames * 1000 < DEAD_RATE * span:
            break
        husky_power_cycle()
    if not counts:
        print("detect_color: 0 (none)")
        return 0
    best = max(counts, key=lambda c: counts[c])
    print("detect_color:", best, ("", "yellow", "red", "green")[best], "| votes", counts)
    return best




if __name__ == "__main__":
    start_gyro()

    straight(190)

    eat(detect_color())

    straight(230)

    eat(detect_color())

    straight(180)

    delay(1500)
    pivot(180)
    report("2열 180 pivot")
    delay(1500)

    straight(50)

    eat(detect_color())

    straight(200)

    # eat(GREEN)

    # straight(340)

    # delay(1500)
    # pivot(-180)
    # report("3열 -180 pivot")
    # delay(1500)

    # eat(YELLOW)

    # straight(300)
    # report("3열")

    # eat(GREEN)

    # straight(400)

    # # 6개 다 먹고 Yellow 뱉기

    # spin(180)

    # straight(-150)
    # # 뒤로 정렬

    # straight(50)

    # spit_yellow()
    # delay(1000)
    
    # straight(160)

    # spit_close()

    # eat(YELLOW)

    # # 존2로 이동
    # spin(-90)
    # straight(-850)
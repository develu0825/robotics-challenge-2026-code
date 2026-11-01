# 응급 키트와 벽을 세우는 C로봇

# 시작 상태 : 응급키트 모터는 수평 방향
# 실행 법 : 첫 왼쪽 버튼은 자이로 영점 조절임(세팅 전 앞 방향으로 두고 왼쪽 버튼 누르기) -> (블루라이트) 왼쪽 버튼 한번 더 누르면 프로그램 시작

# 디버깅 상황 : 앞부분은 다 잘되는 편, 뒷 부분 디버깅이 부족했음

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi

WHEEL_DIAMETER = 55
AXLE_TRACK = 110
LEFT_DIR = Direction.COUNTERCLOCKWISE
RIGHT_DIR = Direction.CLOCKWISE

STRAIGHT_SPEED = 300
STRAIGHT_ACCEL = (400, 250)
TURN_RATE = 150
TURN_ACCEL = (450, 200)

SETTLE_TOL = 0.5
SETTLE_GAIN = 6
SETTLE_MIN_RATE = 20
SETTLE_MAX_RATE = 60
SETTLE_TIMEOUT = 800

MAX_SPEED = 1000
MOVE_MARGIN = 1000

# 벽 내려놓을 때 중간까진 약하게 그 후 빠르게의 임계치 설정
LONG_MID = 20
LONG_UP = 55
LONG_POWER1 = 5
LONG_WAIT = 200
LONG_POWER2 = 10

SHORT_UP = -50
SHORT_POWER = 30
SHORT_MID = -15
SHORT_POWER1 = 5
SHORT_WAIT = 200
SHORT_POWER2 = 10

# 응급 키트 관련 변수 (빠르게 -> 천천히(키트와 직접적으로 닿는 단계) -> 빠르게)
KIT_START = -90
KIT_POWER = 40
KIT_WAIT = 300
KIT_SLOW_START = 60
KIT_SLOW_END = 110
KIT_SLOW_POWER = 10

hub = PrimeHub()
left = Motor(Port.E, LEFT_DIR)
right = Motor(Port.A, RIGHT_DIR)
robot = DriveBase(left, right, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
robot.settings(
    straight_speed=STRAIGHT_SPEED,
    straight_acceleration=STRAIGHT_ACCEL,
    turn_rate=TURN_RATE,
    turn_acceleration=TURN_ACCEL,
)
robot.use_gyro(True)

long_arm = Motor(Port.B)
long_arm.reset_angle(0)
short_arm = Motor(Port.F)
short_arm.reset_angle(0)
kit = Motor(Port.C)
kit_target = KIT_START + 180 * round((kit.angle() - KIT_START) / 180)

target_heading = 0
skip_gyro = False


def start_gyro():
    global target_heading, skip_gyro
    if skip_gyro:
        skip_gyro = False
        return
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


def motor_to(motor, power, target, wait, label):
    speed = max(1, min(100, power)) * MAX_SPEED / 100
    motor.run_target(speed, target, then=Stop.HOLD, wait=False)
    if not wait:
        return
    limit = abs(target - motor.angle()) * 1000 / speed + MOVE_MARGIN
    watch = StopWatch()
    while not motor.done():
        if watch.time() > limit:
            motor.hold()
            print(label, "stopped early at", motor.angle(), "| target", target)
            return
        delay(10)


def long_wall():
    motor_to(long_arm, LONG_POWER1, LONG_MID, True, "long_wall")
    delay(LONG_WAIT)
    motor_to(long_arm, LONG_POWER2, LONG_UP, True, "long_wall")


def short_wall():
    if abs(long_arm.angle() - LONG_UP) > 10:
        print("short_wall: long wall arm is not up, skipped")
        return
    motor_to(short_arm, SHORT_POWER1, SHORT_MID, True, "short_wall")
    delay(SHORT_WAIT)
    motor_to(short_arm, SHORT_POWER2, SHORT_UP, True, "short_wall")


def drop_kit(count=1):
    global kit_target
    for i in range(count):
        if i:
            delay(KIT_WAIT)
        start = kit_target
        motor_to(kit, KIT_POWER, start - KIT_SLOW_START, True, "drop_kit")
        motor_to(kit, KIT_SLOW_POWER, start - KIT_SLOW_END, True, "drop_kit")
        kit_target = start - 180
        motor_to(kit, KIT_POWER, kit_target, True, "drop_kit")


def press_right():
    from pybricks.parameters import Button
    while Button.RIGHT not in hub.buttons.pressed():
        wait(10)
    while Button.RIGHT in hub.buttons.pressed():
        wait(10)


def ready():
    global skip_gyro
    hub.light.on(Color.YELLOW)
    print("press the RIGHT button to reset the gyro")
    press_right()
    start_gyro()
    hub.light.on(Color.BLUE)
    print("press the RIGHT button again to start")
    press_right()
    skip_gyro = True
    hub.light.on(Color.GREEN)

# 기본 함수 모음
# straight(100) : 100만큼 직진 (-는 뒤로)
# spin(90) : 90도 만큼 제자리턴
# pivot(-90) : 피벗턴 -는 왼쪽 바퀴가 멈춤 (forward 생략 가능)
# pivot(90, forward=False) : 피벗턴 오른쪽 바퀴 멈춤 + False일 경우 뒤로
# delay(1000) : 대기 1초
# start_gyro() : 자이로 초기화 가끔 정렬 후에 써도 됨
# report : (사용하지 않아도 됨) 디버깅용 로그 출력 함수, 오차 계산

# C 로봇 함수 모음
# long_wall, short_wall : 긴 벽, 짧은 벽을 내려놓음
# drop_kit(2) : 키트 빼내기 2개

ready()


if __name__ == "__main__":
    start_gyro()

    straight(1000)

    straight(-30)

    pivot(90, forward=False)

    # 첫 노란색 구역에 2개 내려 놓기
    drop_kit(2)

    # 빨간색(병원) 구역까지 후진으로 이동
    straight(-550)

    # 빨간 구역에서 키트 6개
    spin(-30)
    drop_kit(6)
    spin(30)

    straight(-100)

    spin(-90)

    straight(70)

    # 두번째 노란색 구역에 키트 2개
    drop_kit(2)

    straight(-550)

    spin(-90)

    # 벽정렬
    straight(150)

    straight(-430)

    pivot(90)

    straight(-150)

    spin(90)

    straight(-90)

    # 긴 벽 투하
    long_wall()
    delay(500)

    straight(100)

    spin(-90)

    straight(270)

    spin(90)

    straight(-230)

    spin(-90)

    # 긴 벽 밀어 맞추기
    straight(-270)

    straight(150)

    spin(-90)

    straight(150)

    straight(-30)

    spin(90)

    straight(-100)

    # 짧은 벽도 투하
    short_wall()
    delay(500)

    straight(300)
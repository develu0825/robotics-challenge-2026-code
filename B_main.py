# 디스크를 가져다놓는 B로봇
# v1이라 가장 멀리 있는 디스크 하나 잡아오는거일거임 아마

# 시작상태 : 리프트는 다 내리고 그립은 조인 상태
# 실행법 : 1. 왼쪽버튼을 한번 눌러 그립 벌리기 2. 세팅하기 3. 왼쪽 버튼 한 번 더 누르기

# 진행 상황 : 전체적으로 코드 불안정 할수도, main 중간에 주석으로 벽 정렬 후 첫번째까지 두번째까지 세번째까지 값들 있음

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from umath import pi
 
WHEEL_DIAMETER = 55
AXLE_TRACK = 100
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

# 그립
GRIP_POWER = 70
A_CLOSED = 0
A_OPEN = -1280

#리프트
LIFT_POWER = 40
B_DOWN = 5
B_PLACE = -30
# UP 각도는 -330을 넘어가서는 안됨(그럴경우코드수정필요)
B_UP = -180
 
# 디스크 내려놓을 때 좌우로 흔드는거
SHAKE_COUNT = 3
SHAKE_ANGLE = 15
SHAKE_POWER = 30
# 완전히 벌어진채로 흔드는 것이 아니라 값 설정
SHAKE_GRIP = -640
 
hub = PrimeHub()
left = Motor(Port.C, LEFT_DIR)
right = Motor(Port.D, RIGHT_DIR)
robot = DriveBase(left, right, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
robot.settings(
    straight_speed=STRAIGHT_SPEED,
    straight_acceleration=STRAIGHT_ACCEL,
    turn_rate=TURN_RATE,
    turn_acceleration=TURN_ACCEL,
)
robot.use_gyro(True)
 
grip = Motor(Port.A)
grip.reset_angle(A_CLOSED)
lift = Motor(Port.B)
lift.reset_angle(B_DOWN)
 
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
 
 
def grip_open(power=GRIP_POWER, wait=True):
    motor_to(grip, power, A_OPEN, wait, "grip_open")
 
 
def grip_close(power=GRIP_POWER, wait=True):
    motor_to(grip, power, A_CLOSED, wait, "grip_close")
 
 
def lift_up(wait=True):
    motor_to(lift, LIFT_POWER, B_UP, wait, "lift_up")
 
 
def lift_down(wait=True):
    motor_to(lift, LIFT_POWER, B_DOWN, wait, "lift_down")
 
 
def lift_place(wait=True):
    motor_to(lift, LIFT_POWER, B_PLACE, wait, "lift_place")


def shake(count=SHAKE_COUNT, angle=SHAKE_ANGLE, power=SHAKE_POWER):
    from umath import cos
    motor_to(grip, GRIP_POWER, SHAKE_GRIP, True, "shake")
    rate = max(1, min(100, power)) * MAX_SPEED / 100 * WHEEL_DIAMETER / AXLE_TRACK
    w = rate / (1.54 * max(angle, 1))
    period = 2 * pi / w * 1000
    robot.settings(turn_acceleration=min(8000, int(2 * angle * w * w) + 100))
    watch = StopWatch()
    t = 0
    while t < period * count:
        a = angle * (count - int(t // period)) / count
        th = w * t / 1000
        robot.drive(0, a * w * (cos(th) - cos(2 * th)) / 1.299)
        delay(10)
        t = watch.time()
    robot.stop()
    robot.hold()
    robot.settings(turn_acceleration=TURN_ACCEL)
    settle(target_heading)
    motor_to(grip, GRIP_POWER, A_OPEN, True, "shake")


def ready():
    from pybricks.parameters import Button
    hub.light.on(Color.YELLOW)
    print("press the RIGHT button to open the grip")
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)
    grip_open()
    hub.light.on(Color.BLUE)
    print("press the LEFT button to start")
    while Button.LEFT not in hub.buttons.pressed():
        wait(10)
    delay(1000)

# 기본 함수 모음
# straight(100) : 100만큼 직진 (-는 뒤로)
# spin(90) : 90도 만큼 제자리턴
# pivot(-90) : 피벗턴 -는 왼쪽 바퀴가 멈춤 (forward 생략 가능)
# pivot(90, forward=False) : 피벗턴 오른쪽 바퀴 멈춤 + False일 경우 뒤로
# delay(1000) : 대기 1초
# start_gyro() : 자이로 초기화 가끔 정렬 후에 써도 됨
# report : (사용하지 않아도 됨) 디버깅용 로그 출력 함수, 오차 계산

# B 로봇 함수 모음
# grip_open, grip_close : 그립 열고 닫기
# lift_up, lift_down : 리프트 올리고 내리기


ready()


if __name__ == "__main__":
    start_gyro()

    straight(340)

    spin(-90)

    # 벽 정렬
    straight(-100)
    start_gyro()

    straight(600)

    # 첫번째 디스크 잡기
    spin(-180)
    lift_up()
    grip_open()

    straight(-550)

    straight(130)

    spin(90)

    straight(100)

    lift_down()

    straight(50)

    grip_close()
    lift_up()
    
    straight(-150)

    spin(-90)

    # 뒤 벽 정렬
    straight(-200)
    start_gyro()

    # 첫번째:330, 두번째:430 , 세번째:530
    straight(330)

    spin(90)

    straight(170)

    lift_place()
    grip_open()

    straight(-30)

    shake()

    lift_up()
    grip_close()

    # 두번째 디스크 잡으러 가자
    straight(-100)

    spin(45)

    lift_down()
    grip_open()

    straight(200)

    grip_close()
    lift_up()

    straight()
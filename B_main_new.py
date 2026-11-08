# 디스크를 가져다놓는 B로봇
# v2 : (실제로 돌려보진 못함) 첫번째 라인 검사 -> 두번째 라인 검사 이런 느낌
# 함수만 구현되어 있고 main이 아예 안되어있는 급 (메인 앞 부분이나 간격 빼고 갈아엎어야,,)
# v1 + find_disc()
# find_disc() : 그 자리에서 좌우 스캔 -> 디스크 감지 -> 디스크 잡기 -> 원점으로 복귀 (-> 로그에 위치 찍기)

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.pupdevices import ColorSensor
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

GRIP_POWER = 70
A_CLOSED = 0
A_OPEN = -1280

LIFT_POWER = 40
B_DOWN = 5
B_PLACE = -30
B_UP = -180

SHAKE_COUNT = 3
SHAKE_ANGLE = 15
SHAKE_POWER = 30
SHAKE_GRIP = -640

# 디스크 탐지
# 디스크 탐지 라인을 몇 라인으로 할건지
FIND_STEPS = 6
# 디스크 라인 사이 거리
FIND_STEP = 50
# 좌우로 얼마나 움직여서 찾을건지
FIND_SWEEP = 45
# 좌우 탐지 때의 회전 속도 (느릴수록 감지율 높아짐)
FIND_TURN_RATE = 60
# 반사광 임계치는 몇으로 할지
FIND_DARK = 10
# 디스크 감지 후 얼마나 앞으로 갈지(그립 잡기 전)
FIND_APPROACH = 10
FIND_BACK = 30
FIND_GRIP_MID = -640
FIND_PULL = 20
FIND_SQUEEZE = 15

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
color_sensor = ColorSensor(Port.E)

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


def sweep():
    for goal in (-FIND_SWEEP, FIND_SWEEP, 0):
        robot.turn(target_heading + goal - heading(), wait=False)
        while not robot.done():
            if color_sensor.reflection() <= FIND_DARK:
                robot.stop()
                robot.hold()
                return True
            delay(5)
    settle(target_heading)
    return False


def find_disc():
    if abs(grip.angle() - A_OPEN) > 20:
        grip_open()
    robot.settings(turn_rate=FIND_TURN_RATE)
    moved = 0
    found = False
    for line in range(FIND_STEPS):
        if line:
            straight(FIND_STEP)
            moved += FIND_STEP
        if sweep():
            found = True
            break
    robot.settings(turn_rate=TURN_RATE)
    if found:
        print("find_disc: found on line", line + 1, "| angle", round(heading() - target_heading),
              "| reflection", color_sensor.reflection())
        straight(-FIND_BACK)
        lift_down()
        straight(FIND_BACK + FIND_APPROACH)
        motor_to(grip, GRIP_POWER, FIND_GRIP_MID, True, "find_disc")
        straight(-FIND_PULL)
        motor_to(grip, GRIP_POWER, A_CLOSED + FIND_SQUEEZE, True, "find_disc")
        lift_up()
        straight(-(FIND_APPROACH - FIND_PULL))
    else:
        print("find_disc: no disc on", FIND_STEPS, "lines")
    spin(0)
    if moved:
        straight(-moved)
    return found


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


ready()


if __name__ == "__main__":
    start_gyro()

    straight(340)

    spin(-90)

    straight(-100)
    start_gyro()

    straight(600)

    spin(-180)
    lift_up()
    grip_open()

    straight(-550)

    straight(100)

    spin(90)

    find_disc()


    # straight(100)

    # lift_down()

    # straight(50)

    # grip_close()
    # lift_up()

    # straight(-150)

    # spin(-90)

    # # 뒤 벽 정렬
    # straight(-200)
    # start_gyro()

    # # 첫번째:330, 두번째:430 , 세번째:530
    # straight(330)

    # spin(90)

    # straight(170)

    # lift_place()
    # grip_open()

    # straight(-30)

    # shake()

    # lift_up()
    # grip_close()

    # # 두번째 디스크 잡으러 가자
    # straight(-100)

    # spin(45)

    # lift_down()
    # grip_open()

    # straight(200)

    # grip_close()
    # lift_up()

    # straight()

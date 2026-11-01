# [TEST] 모터 각도·바퀴 지름 등 값 측정·캘리브레이션용 파일 (대회 본 실행 파일 아님)
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button, Color, Stop
from pybricks.tools import wait, StopWatch, read_input_byte
from umath import pi

WHEEL_DIAMETER = 55
KIT_START = -90
VERIFY_SPEED = 100
JOG_SPEED = 100
JOG_LIMIT = 270
JOG_KEYS = {"a": -2, "d": 2, "z": -10, "c": 10, "q": -45, "e": 45}

hub = PrimeHub()
left = Motor(Port.E)
right = Motor(Port.A)
long_arm = Motor(Port.B)
short_arm = Motor(Port.F)
kit = Motor(Port.C)
kit_start = kit.angle()
long_arm.reset_angle(0)
short_arm.reset_angle(0)
MOTORS = {"B": long_arm, "F": short_arm}

STEPS = (
    ("LONG_MID", "B", "long wall arm: lift SLOWLY to where the first small lift should stop"),
    ("LONG_UP", "B", "long wall arm: lift to the final position (long wall released)"),
    ("SHORT_UP", "F", "short wall arm: lift to where the short wall is released"),
)

prev_buttons = hub.buttons.pressed()
jog_target = {}


def jog(name, step):
    m = MOTORS[name]
    t = jog_target.get(name)
    if t is None:
        t = m.angle()
    new = t + step
    if abs(new) > JOG_LIMIT:
        print("    jog limit reached")
        return
    jog_target[name] = new
    m.run_target(JOG_SPEED, new, then=Stop.HOLD, wait=False)


def free(name):
    MOTORS[name].stop()
    jog_target.pop(name, None)
    print("    motor", name, "is free, you can turn it by hand")


def wait_input(name, show, fmt, step=2):
    global prev_buttons
    last = None
    watch = StopWatch()
    next_alive = 3000
    while True:
        vals = show()
        if watch.time() >= next_alive:
            print("    ...waiting |", fmt % vals)
            next_alive += 3000
        pressed = hub.buttons.pressed()
        if pressed != prev_buttons:
            new = [b for b in (Button.RIGHT, Button.LEFT) if b in pressed and b not in prev_buttons]
            prev_buttons = pressed
            if new:
                return new[0]
        while True:
            byte = read_input_byte()
            if byte is None:
                break
            key = chr(byte).lower() if 32 <= byte < 127 else ""
            if byte in (10, 13) or key == "r":
                return Button.RIGHT
            if key == "b":
                return Button.LEFT
            if name and key in JOG_KEYS:
                jog(name, JOG_KEYS[key])
            if name and key == "f":
                free(name)
        if last is None or any(abs(a - b) >= step for a, b in zip(vals, last)):
            print("   ", fmt % vals)
            last = vals
        wait(10)


def move(m, speed, target, label):
    m.run_target(speed, target, then=Stop.HOLD, wait=False)
    limit = abs(target - m.angle()) * 1000 / speed + 1500
    watch = StopWatch()
    while not m.done() and watch.time() < limit:
        wait(10)
    m.hold()
    ok = abs(m.angle() - target) <= 5
    print("  ", label, "->", target, "| reached", m.angle(), "" if ok else "<- NOT REACHED, blocked?")
    wait(300)
    return ok


def mm(deg):
    return round(abs(deg) * pi * WHEEL_DIAMETER / 360)


def round5(x):
    return int(5 * round(x / 5))


hub.light.on(Color.YELLOW)
print("c_calibrate | start with both wall arms in the HOLDING position (they become 0)")
print("click this terminal, then type:")
print("  a / d = -2 / +2 deg,  z / c = -10 / +10 deg,  q / e = -45 / +45 deg")
print("  f = free the motor (turn by hand),  r or Enter = record,  b = back / redo")
print("hub RIGHT / LEFT buttons also record / back")

left.stop()
right.stop()
hub.display.char("W")
print("[drive] PUSH the robot FORWARD by hand 20 cm in a straight line (use a ruler), then r")
e0 = left.angle()
a0 = right.angle()
while wait_input(None, lambda: (left.angle() - e0, right.angle() - a0), "E %d | A %d", 30) != Button.RIGHT:
    e0 = left.angle()
    a0 = right.angle()
    print("  reset, push again")
de = left.angle() - e0
da = right.angle() - a0
hub.speaker.beep(1000, 80)
print("  E (left) ", de, "deg")
print("  A (right)", da, "deg")

values = {}
i = 0
while i < len(STEPS):
    name, mname, text = STEPS[i]
    m = MOTORS[mname]
    hub.display.number(i + 1)
    print("[%d/%d] motor %s: %s, then r" % (i + 1, len(STEPS), mname, text))
    if wait_input(mname, lambda: (m.angle(),), mname + " %d") == Button.LEFT:
        i = max(0, i - 1)
        continue
    values[name] = m.angle()
    m.hold()
    jog_target[mname] = values[name]
    hub.speaker.beep(1000, 80)
    print("  recorded", name, "=", values[name])
    i += 1

warnings = 0
print("--- drive ---")
if abs(de) < 90 or abs(da) < 90:
    print("WARNING: a wheel barely turned. push farther, or check the wheel touches the floor")
    warnings += 1
else:
    avg = (abs(de) + abs(da)) / 2
    print("if you pushed exactly 20 cm, wheel diameter is about", round(200 * 360 / (pi * avg), 1), "mm")
    if abs(abs(de) - abs(da)) > 0.25 * max(abs(de), abs(da)):
        print("WARNING: wheels turned different amounts, the push was curved. redo for a clean result")
        warnings += 1
print("--- wall arms ---")
print("B long: 0 -> mid", values["LONG_MID"], "-> up", values["LONG_UP"])
print("F short: 0 -> up", values["SHORT_UP"])
if (values["LONG_MID"] > 0) != (values["LONG_UP"] > 0) or abs(values["LONG_MID"]) >= abs(values["LONG_UP"]):
    print("WARNING: LONG_MID should be between 0 and LONG_UP. redo")
    warnings += 1
print("--- kit motor C ---")
print("C absolute angle at start:", kit_start, "(should be near", KIT_START, "or", KIT_START + 180, ")")
if min(abs(kit_start - KIT_START), abs(kit_start - KIT_START - 180)) > 20:
    print("WARNING: kit arm is not near the start position")
    warnings += 1

hub.light.on(Color.BLUE)
hub.display.char("V")
print("remove the walls. type r (or RIGHT) to test the arms with the motors, b (or LEFT) to skip")
if wait_input(None, lambda: (long_arm.angle(), short_arm.angle()), "B %d | F %d") == Button.RIGHT:
    ok = True
    ok &= move(short_arm, 300, 0, "F down")
    ok &= move(long_arm, 300, 0, "B down")
    ok &= move(long_arm, 50, values["LONG_MID"], "B mid (slow)")
    wait(200)
    ok &= move(long_arm, 100, values["LONG_UP"], "B up")
    ok &= move(short_arm, 300, values["SHORT_UP"], "F up")
    ok &= move(short_arm, 300, 0, "F down")
    ok &= move(long_arm, 300, 0, "B down")
    if not ok:
        warnings += 1

print("type r (or RIGHT) to test ONE kit drop (C turns -180), b (or LEFT) to skip")
if wait_input(None, lambda: (kit.angle(),), "C %d") == Button.RIGHT:
    base = KIT_START + 180 * round((kit.angle() - KIT_START) / 180)
    if not move(kit, 400, base - 180, "C drop"):
        warnings += 1

print("--- paste into c_robot.py ---")
print("LEFT_DIR =", "Direction.CLOCKWISE" if de > 0 else "Direction.COUNTERCLOCKWISE")
print("RIGHT_DIR =", "Direction.CLOCKWISE" if da > 0 else "Direction.COUNTERCLOCKWISE")
for n in ("LONG_MID", "LONG_UP", "SHORT_UP"):
    print(n, "=", round5(values[n]), "| measured", values[n])
hub.light.on(Color.GREEN if warnings == 0 else Color.RED)
hub.display.char("D")
wait(2000)
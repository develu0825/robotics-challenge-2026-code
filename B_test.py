# [TEST] 모터 각도 등 값 측정·캘리브레이션용 파일 (대회 본 실행 파일 아님)
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button, Color, Stop
from pybricks.tools import wait, StopWatch, read_input_byte

HOME_SPEED = 300
HOME_DUTY = 60
VERIFY_SPEED = 500
JOG_SPEED = 200
JOG_MAX = 1800
JOG_KEYS = {"a": -2, "d": 2, "z": -10, "c": 10, "q": -45, "e": 45}

hub = PrimeHub()
grip = Motor(Port.A)

STEPS = (
    ("A_OPEN", "OPEN position to use (a little off the stop, wide enough for the disc)"),
    ("A_GRIP", "put a disc in, close until it JUST holds the disc"),
)

prev_buttons = hub.buttons.pressed()
jog_target = None


def jog(step):
    global jog_target
    t = grip.angle() if jog_target is None else jog_target
    new = min(JOG_MAX, max(0, t + step))
    if new == t:
        print("    limit reached")
        return
    jog_target = new
    grip.run_target(JOG_SPEED, new, then=Stop.HOLD, wait=False)


def free():
    global jog_target
    grip.stop()
    jog_target = None
    print("    motor A is free, you can turn it by hand")


def wait_input():
    global prev_buttons
    last = None
    watch = StopWatch()
    next_alive = 3000
    while True:
        a = grip.angle()
        if watch.time() >= next_alive:
            print("    ...waiting | A", a)
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
            if key in JOG_KEYS:
                jog(JOG_KEYS[key])
            if key == "f":
                free()
        if last is None or abs(a - last) >= 2:
            print("    A", a)
            last = a
        wait(10)


def move(target, label):
    grip.run_target(VERIFY_SPEED, target, then=Stop.HOLD, wait=False)
    limit = abs(target - grip.angle()) * 1000 / VERIFY_SPEED + 1000
    watch = StopWatch()
    while not grip.done() and watch.time() < limit:
        wait(10)
    grip.hold()
    ok = abs(grip.angle() - target) <= 5
    print("  ", label, "->", target, "| reached", grip.angle(), "" if ok else "<- NOT REACHED, blocked?")
    wait(500)
    return ok


def home():
    start = grip.angle()
    grip.run_until_stalled(-HOME_SPEED, then=Stop.COAST, duty_limit=HOME_DUTY)
    return grip.angle() - start


def round5(x):
    return int(5 * round(x / 5))


hub.light.on(Color.YELLOW)
print("grip_calibrate")
print("click this terminal, then type:")
print("  a / d = -2 / +2 deg,  z / c = -10 / +10 deg,  q / e = -45 / +45 deg")
print("  f = free the motor,  r or Enter = record,  b = back / redo")
print("homing: opening A until it hits the stop...")
moved = home()
grip.reset_angle(0)
print("  hit the stop after", moved, "deg -> stop is now A = 0 (positive = closing)")

values = {}
i = 0
while i < len(STEPS):
    name, text = STEPS[i]
    hub.display.number(i + 1)
    print("[%d/%d] %s, then r" % (i + 1, len(STEPS), text))
    if wait_input() == Button.LEFT:
        i = max(0, i - 1)
        continue
    values[name] = grip.angle()
    grip.hold()
    jog_target = values[name]
    hub.speaker.beep(1000, 80)
    print("  recorded", name, "=", values[name])
    i += 1

warnings = 0
gap = values["A_GRIP"] - values["A_OPEN"]
print("--- result ---")
print("open -> grip:", gap, "deg (first measurement was 1281)")
if values["A_OPEN"] < 5:
    print("WARNING: A_OPEN is right on the stop, keep it a few degrees off")
    warnings += 1
if gap < 100:
    print("WARNING: open and grip are too close, redo")
    warnings += 1

hub.light.on(Color.BLUE)
hub.display.char("V")
print("keep the disc ready. type r (or RIGHT) to test, b (or LEFT) to skip")
if wait_input() == Button.RIGHT:
    ok = True
    ok &= move(values["A_OPEN"], "open")
    ok &= move(values["A_GRIP"], "grip")
    ok &= move(values["A_OPEN"], "open")
    print("re-homing check...")
    home()
    err = grip.angle()
    print("  stop found at", err, "deg (should be near 0)")
    if abs(err) > 10:
        print("WARNING: stop position not repeatable, homing may stall early. tell Claude")
        warnings += 1
    if not ok:
        warnings += 1

print("--- paste into disc_robot.py ---")
print("A_OPEN =", round5(values["A_OPEN"]), "| measured", values["A_OPEN"])
print("A_GRIP =", round5(values["A_GRIP"]), "| measured", values["A_GRIP"])
hub.light.on(Color.GREEN if warnings == 0 else Color.RED)
hub.display.char("D")
wait(2000)
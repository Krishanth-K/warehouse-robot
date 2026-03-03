from controller import Robot, Keyboard, Lidar, Motor, Camera

WHEEL_RADIUS = 0.0625
DISTANCE_TO_CENTER = 0.2226

SPEED_MAX = 0.95
SPEED_MIN = -0.3
ANGULAR_SPEED_MAX = 0.3
ANGULAR_SPEED_MIN = -0.3

PIXEL_STEP = 4
FRONT_IMAGE_RATIO = 0.6
CENTER_WIDTH_RATIO = 0.8

OBJECT_DISTANCE_THRESHOLD = 1.8
FRONT_FOV = 0.5
SIDE_FOV = 0.25
HUMAN_ANGLE_MASK = 0.25

CONFIRM_FRAMES = 3
HUMAN_CONFIRM_FRAMES = 3
HUMAN_LOST_FRAMES = 6
HUMAN_REAR_GRACE_FRAMES = 10


def split_lidar(ranges):
    n = len(ranges)
    c = n // 2
    front_start = int(c * (1 - FRONT_FOV))
    front_end   = int(c * (1 + FRONT_FOV))
    front = ranges[front_start:front_end]
    left  = ranges[:int(n * SIDE_FOV)]
    right = ranges[int(n * (1 - SIDE_FOV)):]
    return front, left, right, front_start


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    lidar = robot.getDevice("front_lidar")
    lidar.enable(ts)
    lidar.enablePointCloud()

    camera = robot.getDevice("body_camera")
    camera.enable(ts)

    motor_left = robot.getDevice("middle_left_wheel_joint")
    motor_right = robot.getDevice("middle_right_wheel_joint")
    motor_left.setPosition(float("inf"))
    motor_right.setPosition(float("inf"))

    keyboard = Keyboard()
    keyboard.enable(ts)

    last_state = None
    state_counter = {}

    human_seen_frames = 0
    human_lost_frames = 0
    human_confirmed = False
    human_recent_timer = 0

    print("\nMiR100 perception running \n")

    while robot.step(ts) != -1:

        # ---------- LiDAR ----------
        ranges = lidar.getRangeImage()
        front, left, right, front_offset = split_lidar(ranges)
        front_close = min(front) < OBJECT_DISTANCE_THRESHOLD

        human_ray_index = front.index(min(front)) + front_offset
        human_ratio = human_ray_index / len(ranges)

        side_object = False
        for i, d in enumerate(ranges):
            if d < OBJECT_DISTANCE_THRESHOLD:
                ratio = i / len(ranges)
                if abs(ratio - human_ratio) > HUMAN_ANGLE_MASK:
                    side_object = True
                    break

        # ---------- Camera  ----------
        img = camera.getImage()
        w, h = camera.getWidth(), camera.getHeight()

        y0 = int(h * (1 - FRONT_IMAGE_RATIO))
        x0 = int(w * (0.5 - CENTER_WIDTH_RATIO / 2))
        x1 = int(w * (0.5 + CENTER_WIDTH_RATIO / 2))

        red = green = yellow = False

        for y in range(y0, h, PIXEL_STEP):
            for x in range(x0, x1, PIXEL_STEP):
                r = camera.imageGetRed(img, w, x, y)
                g = camera.imageGetGreen(img, w, x, y)
                b = camera.imageGetBlue(img, w, x, y)

                luminance = 0.2126*r + 0.7152*g + 0.0722*b

                if r > g * 1.3 and r > b * 1.3 and luminance > 30:
                    red = True

                elif g > r * 1.3 and g > b * 1.3 and luminance > 30:
                    green = True

                elif r > 120 and g > 120 and b < 120:
                    yellow = True

        human_camera = (
            (red and green) or
            (green and yellow) or
            (red and green and yellow)
        )

        if human_camera and front_close:
            human_seen_frames += 1
            human_lost_frames = 0
            human_recent_timer = HUMAN_REAR_GRACE_FRAMES
        else:
            human_lost_frames += 1
            human_seen_frames = 0
            human_recent_timer = max(0, human_recent_timer - 1)

        if human_seen_frames >= HUMAN_CONFIRM_FRAMES:
            human_confirmed = True

        if human_lost_frames >= HUMAN_LOST_FRAMES:
            human_confirmed = False

        if human_confirmed or (human_recent_timer > 0 and front_close):
            if side_object:
                state = "Human & Object detected"
            else:
                state = "Human detected"
        else:
            if front_close:
                state = "Object detected"
            else:
                state = "No collision"

        state_counter[state] = state_counter.get(state, 0) + 1
        for k in list(state_counter.keys()):
            if k != state:
                state_counter[k] = 0

        if state_counter[state] >= CONFIRM_FRAMES:
            if state != last_state:
                print(state)
                last_state = state

        key = keyboard.getKey()
        speed = omega = 0.0

        if key == Keyboard.UP:
            speed = SPEED_MAX
        elif key == Keyboard.DOWN:
            speed = SPEED_MIN
        elif key == Keyboard.LEFT:
            omega = ANGULAR_SPEED_MAX
        elif key == Keyboard.RIGHT:
            omega = ANGULAR_SPEED_MIN

        left_v = (speed - omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS
        right_v = (speed + omega * DISTANCE_TO_CENTER) / WHEEL_RADIUS

        motor_left.setVelocity(left_v)
        motor_right.setVelocity(right_v)


if __name__ == "__main__":
    main()

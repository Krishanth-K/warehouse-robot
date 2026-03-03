from controller import Supervisor
import optparse
import math


class Pedestrian(Supervisor):

    def __init__(self):
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 1.0
        self.joints_position_field = []

        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]

        self.height_offsets = [-0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03]

        self.angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]
        ]

        super().__init__()

    def run(self):

        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--trajectory", default="")
        opt_parser.add_option("--speed", type=float, default=0.8)
        options, _ = opt_parser.parse_args()

        if not options.trajectory:
            print("Trajectory required.")
            return

        self.speed = options.speed
        self.time_step = int(self.getBasicTimeStep())

        # Parse waypoints
        points = options.trajectory.split(',')
        waypoints = [[float(p.split()[0]), float(p.split()[1])] for p in points]
        num_points = len(waypoints)

        root = self.getSelf()
        translation_field = root.getField("translation")
        rotation_field = root.getField("rotation")

        for name in self.joint_names:
            self.joints_position_field.append(root.getField(name))

        current_segment = 0
        segment_start_time = 0

        start_delay = 5.0
        walking_started = False
        finished = False
        start_walk_time = 0

        while self.step(self.time_step) != -1:

            sim_time = self.getTime()

            # ===== WAIT BEFORE START =====
            if not walking_started:
                if sim_time < start_delay:
                    continue
                walking_started = True
                start_walk_time = sim_time
                segment_start_time = sim_time

            # ===== IF FINISHED =====
            if finished:
                continue

            if current_segment >= num_points - 1:

                # Final exact position
                x_final, y_final = waypoints[-1]
                translation_field.setSFVec3f([x_final, y_final, self.ROOT_HEIGHT])

                # Face last direction
                x_prev, y_prev = waypoints[-2]
                angle = math.atan2(y_final - y_prev, x_final - x_prev)
                rotation_field.setSFRotation([0, 0, 1, angle])

                # True neutral standing pose
                standing_pose = [0.0] * self.BODY_PARTS_NUMBER
                for i in range(self.BODY_PARTS_NUMBER):
                    self.joints_position_field[i].setSFFloat(standing_pose[i])

                finished = True
                continue

            # ===== NORMAL WALKING =====
            elapsed = sim_time - segment_start_time

            x1, y1 = waypoints[current_segment]
            x2, y2 = waypoints[current_segment + 1]

            segment_length = math.hypot(x2 - x1, y2 - y1)
            segment_duration = segment_length / self.speed

            ratio = elapsed / segment_duration

            if ratio >= 1.0:
                current_segment += 1
                segment_start_time = sim_time
                continue

            # Interpolated position
            x = x1 * (1 - ratio) + x2 * ratio
            y = y1 * (1 - ratio) + y2 * ratio

            angle = math.atan2(y2 - y1, x2 - x1)

            # WALK ANIMATION
            distance_walked = (sim_time - start_walk_time) * self.speed
            cycle = distance_walked / self.CYCLE_TO_DISTANCE_RATIO
            seq = int(cycle) % self.WALK_SEQUENCES_NUMBER
            next_seq = (seq + 1) % self.WALK_SEQUENCES_NUMBER
            interp = cycle - math.floor(cycle)

            for i in range(self.BODY_PARTS_NUMBER):
                a1 = self.angles[i][seq]
                a2 = self.angles[i][next_seq]
                self.joints_position_field[i].setSFFloat(a1 * (1 - interp) + a2 * interp)

            h1 = self.height_offsets[seq]
            h2 = self.height_offsets[next_seq]
            height_offset = h1 * (1 - interp) + h2 * interp

            translation_field.setSFVec3f([x, y, self.ROOT_HEIGHT + height_offset])
            rotation_field.setSFRotation([0, 0, 1, angle])


controller = Pedestrian()
controller.run()
from controller import Supervisor
import optparse
import math


class Pedestrian(Supervisor):

    def __init__(self):
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 0.8
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


    def axis_angle_to_matrix(self, axis, angle):
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1 - c
        return [
            [t*x*x + c,     t*x*y - s*z, t*x*z + s*y],
            [t*x*y + s*z,   t*y*y + c,   t*y*z - s*x],
            [t*x*z - s*y,   t*y*z + s*x, t*z*z + c]
        ]

    def matrix_to_axis_angle(self, m):
        angle = math.acos(max(min((m[0][0] + m[1][1] + m[2][2] - 1) / 2, 1), -1))
        if abs(angle) < 1e-6:
            return [0, 0, 1, 0]
        x = (m[2][1] - m[1][2]) / (2 * math.sin(angle))
        y = (m[0][2] - m[2][0]) / (2 * math.sin(angle))
        z = (m[1][0] - m[0][1]) / (2 * math.sin(angle))
        return [x, y, z, angle]

    def multiply(self, A, B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


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

        points = options.trajectory.split(',')
        waypoints = [[float(p.split()[0]), float(p.split()[1])] for p in points]
        num_points = len(waypoints)

        root = self.getSelf()
        translation_field = root.getField("translation")
        rotation_field = root.getField("rotation")

        for name in self.joint_names:
            self.joints_position_field.append(root.getField(name))

        qr_front = self.getFromDef("QR_FRONT")
        qr_back = self.getFromDef("QR_BACK")

        qr_front_translation = qr_front.getField("translation")
        qr_front_rotation = qr_front.getField("rotation")
        qr_back_translation = qr_back.getField("translation")
        qr_back_rotation = qr_back.getField("rotation")

        ped_initial = translation_field.getSFVec3f()
        front_initial = qr_front_translation.getSFVec3f()
        back_initial = qr_back_translation.getSFVec3f()

        front_offset = [front_initial[i] - ped_initial[i] for i in range(3)]
        back_offset = [back_initial[i] - ped_initial[i] for i in range(3)]

        front_matrix = self.axis_angle_to_matrix(qr_front_rotation.getSFRotation()[:3],
                                                 qr_front_rotation.getSFRotation()[3])
        back_matrix = self.axis_angle_to_matrix(qr_back_rotation.getSFRotation()[:3],
                                                qr_back_rotation.getSFRotation()[3])

        current_segment = 0
        segment_start_time = 0
        start_delay = 5.0
        walking_started = False
        finished = False
        start_walk_time = 0

        waypoint_pause_segment = 0  
        waypoint_pause_duration = 11.0
        waypoint_pausing = False
        waypoint_pause_start = 0
        waypoint_pause_done = False

        pause_index = 2
        pause_duration = 3.0
        is_pausing = False
        pause_start_time = 0

        while self.step(self.time_step) != -1:

            sim_time = self.getTime()

            if not walking_started:
                if sim_time < start_delay:
                    continue
                walking_started = True
                start_walk_time = sim_time
                segment_start_time = sim_time

            if finished:
                continue

            x1, y1 = waypoints[current_segment]
            x2, y2 = waypoints[current_segment + 1]

            segment_length = math.hypot(x2 - x1, y2 - y1)
            segment_duration = segment_length / self.speed
            elapsed = sim_time - segment_start_time
            ratio = elapsed / segment_duration

            if ratio >= 1.0:

                translation_field.setSFVec3f([x2, y2, self.ROOT_HEIGHT])

                # ---------- NEW WAYPOINT PAUSE ----------
                if current_segment == waypoint_pause_segment and not waypoint_pause_done:
                    if not waypoint_pausing:
                        waypoint_pausing = True
                        waypoint_pause_start = sim_time
                        for i in range(self.BODY_PARTS_NUMBER):
                            self.joints_position_field[i].setSFFloat(0.0)
                    elif sim_time - waypoint_pause_start >= waypoint_pause_duration:
                        waypoint_pausing = False
                        waypoint_pause_done = True
                        current_segment += 1
                        segment_start_time = sim_time
                    continue

                # ---------- EXISTING PAUSE ----------
                if current_segment + 1 == pause_index:
                    if not is_pausing:
                        is_pausing = True
                        pause_start_time = sim_time
                        for i in range(self.BODY_PARTS_NUMBER):
                            self.joints_position_field[i].setSFFloat(0.0)
                    elif sim_time - pause_start_time >= pause_duration:
                        is_pausing = False
                        current_segment += 1
                        segment_start_time = sim_time
                    continue

                current_segment += 1
                segment_start_time = sim_time

                if current_segment >= num_points - 1:
                    for i in range(self.BODY_PARTS_NUMBER):
                        self.joints_position_field[i].setSFFloat(0.0)
                    finished = True
                continue

            if is_pausing or waypoint_pausing:
                continue

            x = x1*(1-ratio) + x2*ratio
            y = y1*(1-ratio) + y2*ratio
            angle = math.atan2(y2 - y1, x2 - x1)

            distance_walked = (sim_time - start_walk_time) * self.speed
            cycle = distance_walked / self.CYCLE_TO_DISTANCE_RATIO
            seq = int(cycle) % self.WALK_SEQUENCES_NUMBER
            next_seq = (seq + 1) % self.WALK_SEQUENCES_NUMBER
            interp = cycle - math.floor(cycle)

            for i in range(self.BODY_PARTS_NUMBER):
                a1 = self.angles[i][seq]
                a2 = self.angles[i][next_seq]
                self.joints_position_field[i].setSFFloat(a1*(1-interp)+a2*interp)

            h1 = self.height_offsets[seq]
            h2 = self.height_offsets[next_seq]
            height_offset = h1*(1-interp)+h2*interp

            translation_field.setSFVec3f([x, y, self.ROOT_HEIGHT + height_offset])
            rotation_field.setSFRotation([0, 0, 1, angle])

            ped_pos = translation_field.getSFVec3f()
            ped_rot = rotation_field.getSFRotation()
            ped_matrix = self.axis_angle_to_matrix(ped_rot[:3], ped_rot[3])

            cos_a = math.cos(ped_rot[3])
            sin_a = math.sin(ped_rot[3])

            fx = ped_pos[0] + front_offset[0]*cos_a - front_offset[1]*sin_a
            fy = ped_pos[1] + front_offset[0]*sin_a + front_offset[1]*cos_a
            fz = ped_pos[2] + front_offset[2]
            qr_front_translation.setSFVec3f([fx, fy, fz])
            qr_front_rotation.setSFRotation(
                self.matrix_to_axis_angle(self.multiply(ped_matrix, front_matrix))
            )

            bx = ped_pos[0] + back_offset[0]*cos_a - back_offset[1]*sin_a
            by = ped_pos[1] + back_offset[0]*sin_a + back_offset[1]*cos_a
            bz = ped_pos[2] + back_offset[2]
            qr_back_translation.setSFVec3f([bx, by, bz])
            qr_back_rotation.setSFRotation(
                self.matrix_to_axis_angle(self.multiply(ped_matrix, back_matrix))
            )


controller = Pedestrian()
controller.run()

from controller import Supervisor
import math


class Pedestrian(Supervisor):
    def __init__(self):
        Supervisor.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Joint names for Pedestrian model (13 joints)
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ] 

        self.joints = []
        root = self.getSelf()
        self.root_rotation_field = root.getField("rotation")

        # Fetch joint fields
        for name in self.joint_names:
            field = root.getField(name)
            if field is None:
                raise RuntimeError(f"Joint '{name}' not found in Pedestrian PROTO")
            self.joints.append(field)

        # State flags 
        self.rotated = False
        self.hands_raised = False

    def run(self):
        while self.step(self.time_step) != -1:
            t = self.getTime()

            # -------------------------
            # 1. Rotate 90° to THE RIGHT (once)
            # -------------------------
            if not self.rotated:
                current = self.root_rotation_field.getSFRotation()
                new_angle = current[3] - math.pi/2  # right turn
                self.root_rotation_field.setSFRotation([0, 0, 1, new_angle])
                self.rotated = True
                continue

            # -------------------------
            # 2. Raise hands 60° forward (once)
            # -------------------------
            if not self.hands_raised:
                forward = -1.0471975512  # -60 degrees → forward in Pedestrian
                self.joints[0].setSFFloat(forward)   # left upper arm
                self.joints[3].setSFFloat(forward)   # right upper arm
                self.joints[1].setSFFloat(0.2)       # bend elbows
                self.joints[4].setSFFloat(0.2)
                self.hands_raised = True
                continue

            # -------------------------
            # 3. Shake hands continuously
            # -------------------------
            shake = 0.05 * math.sin(t * 6.0)  # small +/- 3° shaking
            base = -1.0471975512              # forward 60°

            # Opposite-phase shaking
            self.joints[0].setSFFloat(base + shake)   # left arm
            self.joints[3].setSFFloat(base - shake)   # right arm

            # Keep elbows bent
            self.joints[1].setSFFloat(0.2)
            self.joints[4].setSFFloat(0.2)


controller = Pedestrian()
controller.run()

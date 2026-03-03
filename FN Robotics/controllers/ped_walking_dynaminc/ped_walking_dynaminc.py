from controller import Supervisor
import optparse
import math


class Pedestrian(Supervisor):

    def __init__(self):

        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22

        self.speed = 1.75
        self.distance_walked = 0.0
        self.avoid_timer = 0.0

        self.joints_position_field = []

        self.joint_names = [
            "leftArmAngle","leftLowerArmAngle","leftHandAngle",
            "rightArmAngle","rightLowerArmAngle","rightHandAngle",
            "leftLegAngle","leftLowerLegAngle","leftFootAngle",
            "rightLegAngle","rightLowerLegAngle","rightFootAngle",
            "headAngle"
        ]

        self.height_offsets = [-0.02,0.04,0.08,-0.03,-0.02,0.04,0.08,-0.03]

        self.angles = [
            [-0.52,-0.15,0.58,0.7,0.52,0.17,-0.36,-0.74],
            [0.0,-0.16,-0.7,-0.38,-0.47,-0.3,-0.58,-0.21],
            [0.12,0.0,0.12,0.2,0.0,-0.17,-0.25,0.0],
            [0.52,0.17,-0.36,-0.74,-0.52,-0.15,0.58,0.7],
            [-0.47,-0.3,-0.58,-0.21,0.0,-0.16,-0.7,-0.38],
            [0.0,-0.17,-0.25,0.0,0.12,0.0,0.12,0.2],
            [-0.55,-0.85,-1.14,-0.7,-0.56,0.12,0.24,0.4],
            [1.4,1.58,1.71,0.49,0.84,0.0,0.14,0.26],
            [0.07,0.07,-0.07,-0.36,0.0,0.0,0.32,-0.07],
            [-0.56,0.12,0.24,0.4,-0.55,-0.85,-1.14,-0.7],
            [0.84,0.0,0.14,0.26,1.4,1.58,1.71,0.49],
            [0.0,0.0,0.42,-0.07,0.07,0.07,-0.07,-0.36],
            [0.18,0.09,0.0,0.09,0.18,0.09,0.0,0.09]
        ]

        super().__init__()

    # ---------- ORIGINAL QR ROTATION UTILITIES ----------
    def axis_angle_to_matrix(self, axis, angle):
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1 - c
        return [
            [t*x*x + c, t*x*y - s*z, t*x*z + s*y],
            [t*x*y + s*z, t*y*y + c, t*y*z - s*x],
            [t*x*z - s*y, t*y*z + s*x, t*z*z + c]
        ]

    def matrix_to_axis_angle(self, m):
        angle = math.acos(max(min((m[0][0]+m[1][1]+m[2][2]-1)/2,1),-1))
        if abs(angle) < 1e-6:
            return [0,0,1,0]
        x=(m[2][1]-m[1][2])/(2*math.sin(angle))
        y=(m[0][2]-m[2][0])/(2*math.sin(angle))
        z=(m[1][0]-m[0][1])/(2*math.sin(angle))
        return [x,y,z,angle]

    def multiply(self,A,B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    # ---------- LIDAR ----------
    def lidar_blocked(self,lidar):

        ranges = lidar.getRangeImage()
        if not ranges:
            return False

        n=len(ranges)
        start=int(n*0.4)
        end=int(n*0.6)

        min_dist=999
        for i in range(start,end):
            r=ranges[i]
            if r>0:
                min_dist=min(min_dist,r)

        return min_dist<0.4

    # ---------- AVOIDANCE ----------
    def create_avoidance_point(self,pos,angle):

        offset=0.7
        side_x=-math.sin(angle)*offset
        side_y= math.cos(angle)*offset

        return [pos[0]+side_x,pos[1]+side_y]

    # ---------- WALK ANIMATION ----------
    def update_animation(self):

        cycle=self.distance_walked/self.CYCLE_TO_DISTANCE_RATIO

        seq=int(cycle)%self.WALK_SEQUENCES_NUMBER
        next_seq=(seq+1)%self.WALK_SEQUENCES_NUMBER
        interp=cycle-math.floor(cycle)

        for i in range(self.BODY_PARTS_NUMBER):
            a1=self.angles[i][seq]
            a2=self.angles[i][next_seq]
            self.joints_position_field[i].setSFFloat(a1*(1-interp)+a2*interp)

        h1=self.height_offsets[seq]
        h2=self.height_offsets[next_seq]

        return h1*(1-interp)+h2*interp

    # ---------- MAIN ----------
    def run(self):

        opt_parser=optparse.OptionParser()
        opt_parser.add_option("--trajectory",default="")
        opt_parser.add_option("--speed",type=float,default=1.75)
        options,_=opt_parser.parse_args()

        self.speed=options.speed
        self.time_step=int(self.getBasicTimeStep())
        dt=self.time_step/1000.0

        lidar=self.getDevice("lidar")
        if lidar:
            lidar.enable(self.time_step)
            lidar.enablePointCloud()

        points=options.trajectory.split(',')
        waypoints=[[float(p.split()[0]),float(p.split()[1])] for p in points]

        root=self.getSelf()
        translation_field=root.getField("translation")
        rotation_field=root.getField("rotation")

        for name in self.joint_names:
            self.joints_position_field.append(root.getField(name))

        # ---------- ORIGINAL QR SETUP ----------
        qr_front=self.getFromDef("QR_FRONT")
        qr_back=self.getFromDef("QR_BACK")

        qr_front_translation=qr_front.getField("translation")
        qr_front_rotation=qr_front.getField("rotation")
        qr_back_translation=qr_back.getField("translation")
        qr_back_rotation=qr_back.getField("rotation")

        ped_initial=translation_field.getSFVec3f()
        front_initial=qr_front_translation.getSFVec3f()
        back_initial=qr_back_translation.getSFVec3f()

        front_offset=[front_initial[i]-ped_initial[i] for i in range(3)]
        back_offset=[back_initial[i]-ped_initial[i] for i in range(3)]

        front_matrix=self.axis_angle_to_matrix(qr_front_rotation.getSFRotation()[:3],
                                               qr_front_rotation.getSFRotation()[3])
        back_matrix=self.axis_angle_to_matrix(qr_back_rotation.getSFRotation()[:3],
                                              qr_back_rotation.getSFRotation()[3])

        current_target_index=0

        while self.step(self.time_step)!=-1:

            if current_target_index>=len(waypoints):
                continue

            pos=translation_field.getSFVec3f()
            target=waypoints[current_target_index]

            dx=target[0]-pos[0]
            dy=target[1]-pos[1]
            dist=math.hypot(dx,dy)

            if dist<0.05:
                current_target_index+=1
                continue

            angle=math.atan2(dy,dx)

            self.avoid_timer-=dt

            if lidar and self.lidar_blocked(lidar) and self.avoid_timer<=0:
                avoid=self.create_avoidance_point(pos,angle)
                waypoints.insert(current_target_index,avoid)
                self.avoid_timer=1.0
                continue

            step=min(self.speed*dt,dist)
            self.distance_walked+=step

            height_offset=self.update_animation()

            new_x=pos[0]+math.cos(angle)*step
            new_y=pos[1]+math.sin(angle)*step

            translation_field.setSFVec3f([new_x,new_y,self.ROOT_HEIGHT+height_offset])
            rotation_field.setSFRotation([0,0,1,angle])

            # ---------- ORIGINAL QR UPDATE ----------
            ped_pos=translation_field.getSFVec3f()
            ped_rot=rotation_field.getSFRotation()
            ped_matrix=self.axis_angle_to_matrix(ped_rot[:3],ped_rot[3])

            cos_a=math.cos(ped_rot[3])
            sin_a=math.sin(ped_rot[3])

            fx=ped_pos[0]+front_offset[0]*cos_a-front_offset[1]*sin_a
            fy=ped_pos[1]+front_offset[0]*sin_a+front_offset[1]*cos_a
            fz=ped_pos[2]+front_offset[2]
            qr_front_translation.setSFVec3f([fx,fy,fz])
            qr_front_rotation.setSFRotation(
                self.matrix_to_axis_angle(self.multiply(ped_matrix,front_matrix))
            )

            bx=ped_pos[0]+back_offset[0]*cos_a-back_offset[1]*sin_a
            by=ped_pos[1]+back_offset[0]*sin_a+back_offset[1]*cos_a
            bz=ped_pos[2]+back_offset[2]
            qr_back_translation.setSFVec3f([bx,by,bz])
            qr_back_rotation.setSFRotation(
                self.matrix_to_axis_angle(self.multiply(ped_matrix,back_matrix))
            )


controller=Pedestrian()
controller.run()

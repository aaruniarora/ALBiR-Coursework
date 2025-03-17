from servos import *
from camera import *
from pid_control import PID
import time, sensor, math, random

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 25, p=1, i=0.015, d=0.005, psteer=1, isteer=0, dsteer=0.005, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)
        self.PID_steering = PID(psteer, isteer, dsteer, imax=0)
        self.thresholds = thresholds

        # Blob IDs: blue (1), red obstacle (2), green (4), left (8), right (16)
        self.blue = {'id': 0, 'code': pow(2,0), 'pix_thresh': 750, 'area_thresh': 750}
        self.obstacle = {'id': 1, 'code': pow(2,1), 'pix_thresh': 1000, 'area_thresh': 1000}
        self.green = {'id': 2, 'code': pow(2,2), 'pix_thresh': 500, 'area_thresh': 500}
        self.left = {'id': 3, 'code': pow(2,3), 'pix_thresh': 1000, 'area_thresh': 1000}
        self.right = {'id': 4, 'code': pow(2,4), 'pix_thresh': 1000, 'area_thresh': 1000}

        self.scan_direction = 1
        self.global_pos = {'x': 0, 'y': 0}


    def get_snap(self, colour_code, pix_thresh=100, area_thresh=100, closest=False, rot_angle=False, debug_mode=False):
        """
        Takes a snapshot and returns the biggest blob for the specified colour (if closest=False).

        Args:
            colour_code (int): Colour code of the blob to return.
            pix_thresh (int): Minimum size of the blob in pixels.
            area_thresh (int): Minimum area of the blob.
            closest (bool): If True, returns the closest blob (not necessarily the biggest) of the
                                specified colour
            debug_mode (bool): If True, displays rectangles around all detected blobs of all colours
        """
        if rot_angle:
            blobs=[b for b in self.cam.get_blobs(angle=self.servo.pan_pos, pix_thresh=pix_thresh,
                    area_thresh=area_thresh)[0] if b.code()==colour_code]
        else:
            blobs=[b for b in self.cam.get_blobs(pix_thresh=pix_thresh, area_thresh=area_thresh)[0]
                    if b.code()==colour_code]

        if debug_mode:
            img = sensor.snapshot()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh, area_thresh=area_thresh):
                img.draw_rectangle(blob.rect())
                img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)

        if blobs and closest:
            closest_blob = sorted(blobs, key=lambda b: b.cy())
            return closest_blob[-1]

        return self.cam.get_biggest_blob(blobs)


    def track_blob(self, blob, pan=True) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        if pan:
            # Move pan servo to track block
            self.servo.set_angle(pan_angle)

        return pan_angle


    def scan_all(self, targets_list: list, step = 2, limit = 65, scan_bias=1, pix_thresh=200, area_thresh=200) -> None:
        """
        Scans left and right with the camera to find the blobs.
        """
        lost_count = 0

        # scan_bias=0.7 means 70% chance for 1 (right), 30% chance for -1 (left)
        self.scan_direction = 1 if random.uniform(0, 1) < scan_bias else -1

        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Identify the relevant blob of current color
            img = sensor.snapshot()

            for target in targets_list:
                target_blob = self.get_snap(target['code'], pix_thresh=target['pix_thresh'], area_thresh=target['area_thresh'])
                if target_blob:
                    print('In Scanning at pan', new_pan_angle, 'and I see threshold idx', target['id'])
                    return target_blob, new_pan_angle
                    break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                print("When scanning",  self.servo.pan_pos )
                lost_count += 1
                self.scan_direction *= -1

            if lost_count > 1:
                print("Didn't find anything :(")
                self.servo.set_angle(0)
                return None, self.servo.pan_pos
                break


    def scan_for_blob(self, target: dict, step = 2, limit = 65, pix_thresh=200, area_thresh=200) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        lost_count = 0
        print('activated scan_for_blob')

        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Identify the relevant blob of current color
            img = sensor.snapshot()

            if target_blob:=self.get_snap(target['code'], pix_thresh=target['pix_thresh'], area_thresh=target['area_thresh']):
                print('In scanning at pan', new_pan_angle, 'and I see threshold idx', target['id'])
                return target_blob, new_pan_angle
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                print("When scanning",  self.servo.pan_pos )
                lost_count += 1
                self.scan_direction *= -1

            if lost_count > 1:
                print("Didn't find anything while scanning :(")
                self.servo.set_angle(0)
                return None, self.servo.pan_pos
                break


    def drive(self, drive: float, steering: float) -> None:
        """Differential drive control for the robot."""
        self.servo.set_differential_drive(drive, steering)


    def move_offset(self, target_blob_cx, speed=0.1):
        pixel_offset = target_blob_cx - self.cam.w_centre
        steering = self.PID_steering.get_pid(pixel_offset/sensor.width(), scaler=1.0)
        self.drive(speed, steering) # Move pan servo to track block
        # print('steering:', steering, 'offset:', pixel_offset)


    def move_bias(self, target_blob, speed=0.1, t=100, obs=False, control=False):
        pan_angle = self.track_blob(target_blob)
        steering = self.PID_steering.get_pid(pan_angle, scaler=1.0)*(-1/90)

        if control:
            if abs(pan_angle) < 1:
                self.drive(speed, 0.0)
            else:
                self.drive(speed, steering)

        if obs:
            self.drive(speed, -steering)
            print('In move_bias for obs. Steering at', steering, 'pan pos', self.servo.pan_pos, 'for time', t)
        else:
            self.drive(speed, steering)
            print('In move_bias normal. Steering at', steering, 'pan pos', self.servo.pan_pos, 'for time', t)

        time.sleep_ms(t)


    def get_distance(self, blob, shape='square'):
        if shape == 'obstacle':
            y = blob.h()/2 + blob.cy()
        else:
            y = blob.cy()
        # return 1.1614*10**(-8) * y**4 - 9.09*10**(-6) * y**3 + 0.0021958 * y**2 - 0.28977 * y**1 + 29.4503 * y**0
        return 4.3836 * 10**(-9) * y**4 - 3.9621* 10**(-6) * y**3 + 0.0013057 * y**2 - 0.24872 * y**1 + 31.7243 * y**0


    def move_check(self, speed: float) -> None:
        """
        Checks servo pan (cup method)
        """
        at_end = False
        while not at_end:
            img = sensor.snapshot()

            obstacle = self.get_snap(self.obstacle_code, pix_thresh=1000, area_thresh=1000) #, closest=True)

            if obstacle is None:
                print('Searching')
                self.scan_for_blob(threshold_idx=self.obstacle_id, step = 2, limit = 75, pix_thresh=200, area_thresh=200)
                print('Searched')

            elif obstacle:
                print("I spy with my little eye an obstacle at", obstacle.cx())
                self.move_bias(obstacle, speed)

            else:
                print("I can't see anything, so I'll stop")
                at_end = True

        self.servo.soft_reset()
        return


    def align_body(self, target, scan_bias, speed=0.08):
        self.servo.set_speed(0, 0)
        time.sleep_ms(50)
        count = 0

        while True:
            img = sensor.snapshot()
            target_blob = self.get_snap(target['code'], pix_thresh=target['pix_thresh'], area_thresh=target['area_thresh'], closest=True)

            if target_blob:
                pan_angle = self.track_blob(target_blob)
                if abs(pan_angle) < 10:
                    # self.servo.set_angle(0)
                    print("Aligned")
                    break
                else:
                    if pan_angle > 0: # blob on right, turn right
                        print('Blob on right, aligning')
                        self.servo.set_speed(speed, -speed)
                        time.sleep_ms(70)
                    else: # blob on left, turn left
                        print('Blob on left, aligning')
                        self.servo.set_speed(-speed, speed)
                        time.sleep_ms(70)
                    self.servo.set_speed(0, 0)
                    time.sleep_ms(50)
            else:
                print('in align blob searching for', count)
                if count == 0:
                    target_blob, pan_angle = self.scan_all([target], step=2, limit=30, scan_bias=scan_bias)
                    if target_blob:
                        self.servo.set_angle((pan_angle/abs(pan_angle))*(abs(pan_angle)+3))
                elif count == 1:
                    self.double_check_snap([target], angle=40, check=3)
                elif count == 2:
                    self.double_check_snap([target], angle=-40, check=3)
                elif count == 3:
                    self.double_check_snap([target], angle=0, check=3)
                else:
                    break
                count += 1



    def turn_on_spot(self, scan_bias, speed=0.08, num=20):
        # scan_bias=0.7 means 70% chance for 1 (right), 30% chance for -1 (left)
        self.scan_direction = 1 if random.uniform(0, 1) < scan_bias else -1

        for i in range(num):
            print("Turning on spot for", i)
            self.servo.set_speed(self.scan_direction*speed, -self.scan_direction*speed)
            time.sleep_ms(150)
            self.servo.set_speed(0, 0)
            time.sleep_ms(100)

            green_blob = self.get_snap(self.green['code'], pix_thresh=self.green['pix_thresh'], area_thresh=self.green['area_thresh'])
            blue_blob = self.get_snap(self.blue['code'], pix_thresh=self.blue['pix_thresh'], area_thresh=self.blue['area_thresh'])

            if green_blob or blue_blob:
                print("I see a blob in turn on spot")
                return True


    def double_check_snap(self, targets_list, angle=0, check=5):
        self.servo.set_angle(angle)
        for i in range(check):
            for target in targets_list:
                print(f"Making sure target {target["id"]} is not there for {i}")
                target_blob = self.get_snap(target["code"], pix_thresh=target["pix_thresh"], area_thresh=target["area_thresh"])
                if target_blob:
                    print("Found target id", target["id"], "in double check")
                    return target_blob
        return None


    def maze_solver(self, speed: float, scan_bias: float, stop_dist: float = 8):

        print('Oh easy! I can solve this maze.')

        at_end = False
        lost_count = 0

        while not at_end:
            img = sensor.snapshot()

            ## Detect color blobs
            blue_box     = self.get_snap(self.blue['code'], pix_thresh=self.blue['pix_thresh'], area_thresh=self.blue['area_thresh'], closest=True)
            obstacle     = self.get_snap(self.obstacle['code'], pix_thresh=self.obstacle['pix_thresh'], area_thresh=self.obstacle['area_thresh'])
            finish_blob  = self.get_snap(self.green['code'], pix_thresh=self.green['pix_thresh'], area_thresh=self.green['area_thresh'])
            left_line    = self.get_snap(self.left['code'], pix_thresh=self.left['pix_thresh'], area_thresh=self.left['area_thresh'])
            right_line   = self.get_snap(self.right['code'], pix_thresh=self.right['pix_thresh'], area_thresh=self.right['area_thresh'])
            # print('left', left_line)
            # print('right', right_line)

            ## If I see the Green Square
            if finish_blob:
                lost_count = 0
                _ = self.track_blob(finish_blob)
                green_dist = self.get_distance(finish_blob)
                print("Finish Dist:", green_dist, "cm")

                # If very close to finish, stop
                if green_dist <= stop_dist:
                    self.align_body(self.green, scan_bias=scan_bias)
                    self.servo.set_speed(speed,speed)
                    time.sleep_ms(1500)
                    print("Reached finish!")
                    at_end = True
                    continue  # end the main loop

                else:
                    self.move_bias(finish_blob, speed=speed, t=100)

            ## If I see Blue Square
            elif blue_box:
                lost_count = 0
                pan_angle = self.track_blob(blue_box)
                curr_dist = self.get_distance(blue_box)
                print("Blue Dist:", curr_dist, "cm and pixels", blue_box.cy())

                # If close enough, drive blindly and consider it "reached"
                if curr_dist <= stop_dist:
                    self.align_body(self.blue, scan_bias=scan_bias)
                    self.servo.set_speed(speed,speed)
                    time.sleep_ms(1500)
                    print("Reached blue box!")

                    self.servo.set_speed(0, 0)
                    time.sleep(0.05)
                    curr_dist = 10
                    continue

                else:
                    self.move_bias(blue_box, speed=speed, t=100)


            ## If I don't see anything, either there's an obstacle or I'm at an edge
            else:
                # Repulsion from obstacle
                if obstacle:
                    obs_dist = self.get_distance(obstacle, shape='obstacle')
                    print('Obstacle dist:', obs_dist, 'pixel:', obstacle.cy())
                    if obs_dist <= 6 and (self.servo.curr_l_speed != 0 or self.servo.curr_r_speed != 0):
                        print('Eww obstacle!')
                        self.servo.set_speed(0,0)
                        time.sleep_ms(50)
                        if obstacle.cy() >= self.cam.w_centre:
                            self.turn_on_spot(scan_bias=0,num=2)
                        else:
                            self.turn_on_spot(scan_bias=1,num=2)

                if left_line and self.get_distance(left_line) < stop_dist:
                    print('Left line, so turning right')
                    self.turn_on_spot(scan_bias=1, num=30)
                    continue

                elif right_line and self.get_distance(right_line) < stop_dist:
                    print('Right line, so turning left')
                    self.turn_on_spot(scan_bias=0, num=30)
                    continue

                print("I don't see anything")
                self.servo.set_speed(0,0)
                blob = self.double_check_snap([self.blue, self.green])

                if not blob:
                    lost_count += 1
                    print("I don't see anything after double check for", lost_count)
                    blob, pan_angle = self.scan_all([self.blue, self.green], step=5, limit=90, scan_bias=scan_bias)
                    if blob:
                        # Move pan servo by 5 to track block
                        angle = (pan_angle/abs(pan_angle))*(abs(pan_angle)+3)
                        self.servo.set_angle(angle)
                        self.double_check_snap([self.blue, self.green], angle=angle, check=3)
                        print('Found after scanning at pan', self.servo.pan_pos)
                        continue
                    else:
                        print('Entering turning on spot')
                        self.servo.set_angle(0)
                        line_blob, _ = self.scan_all([self.left, self.right], step=5, limit=20, scan_bias=scan_bias)

                        if line_blob and line_blob.code() == self.left['code']:
                            print('Spot turn right')
                            self.turn_on_spot(scan_bias=0)
                        elif line_blob and line_blob.code() == self.right['code']:
                            print('Spot turn left')
                            self.turn_on_spot(scan_bias=1)
                        else:
                            print('Spot turn random')
                            self.turn_on_spot(scan_bias=scan_bias)

                if lost_count > 5:
                    print("I'm sorry, I got cocky and failed.")
                    break

        self.servo.soft_reset()
        return


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


if __name__ == "__main__":

    # Threshold order: blue (1), red obstacle (2), green (4), left (8), right (16)
    thresholds = [
                    (34, 45, -5, 12, -39, -20), # blue 15 301C
                    (20, 53, 27, 53, 0, 34), # red 15 301C
                    (24, 39, -32, -13, -12, 27), # green 15 301C
                    (34, 54, 18, 36, -19, -2), # left pink 15 AA
                    (78, 91, -21, 9, 20, 64), # right yellow 15 AA

                    # (46, 67, -11, 5, -40, -19), # blue 25 301C
                    # (51, 75, 24, 52, 1, 45), # red 25 301C
                    # (86, 97, -44, -15, 9, 41), # green 25 301C

                    # (17, 35, -10, 10, -35, -13), # Blue 25 AA
                    # (17, 35, 15, 40, 0, 32), # Red 25 AA
                    # (10, 30, -28, -14, 4, 25), # Green 25 AA
                    # (45, 72, -21, 3, 16, 50), # left yellow 25 AA
                    # (30, 37, 15, 28, -13, 5), # right pink 25 AA

                    # (32, 41, -9, 10, -36, -14), # Blue 301D 15
                    # (24, 52, 27, 48, 5, 36), # Red 301D 15
                    # (36, 60, -35, -15, 0, 28), # Green 301D 15
                    # (69, 87, -14, 7, 17, 39), # Left yellow 301D 15
                    # (39, 52, 19, 36, -20, -5), # right pink 301D 15


                    # (40, 80, -11, 8, -45, -18), # Blue 18 301D
                    # (52, 75, 17, 40, -16, 12), # Red 18 301D
                    ]

    # AA gain 20; PP gain 10; 301D gain 25
    robot = Robot(thresholds, gain = 15, p=1, i=0.015, d=0.005, psteer=1, isteer=0, dsteer=0.0, imax=0.01)
    time.sleep(1) # for gain to activate

    motor_cam_dist = -3
    motor_front_dist = 0.5

    # robot.move_check(0.1)

    ## scan_bias=0.7 means 70% chance for 1 (right), 30% chance for -1 (left)
    robot.maze_solver(speed=0.1, scan_bias=0.5, stop_dist=8)

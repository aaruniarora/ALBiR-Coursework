from servos import *
from camera import *
from pid_control import PID
import time, sensor, math

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 15, p=1.1, i=0.015, d=0.005, imax=0.01):
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
        self.PID_steering = PID(1.5, 0.0, 0.01, imax=0)
        self.thresholds = thresholds

        # Blob IDs
        self.mid_line_id = 0
        self.l_line_id = 1
        self.r_line_id = 2
        self.obstacle_id = 3

        self.scan_direction = 1


    # def find_my_blobs(self, img):
    #     # Threshold order: red_line, left_line, right_line, obstacle, white_blob
    #     red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=200, area_threshold=200)
    #     # for l_blob in img.find_blobs(self.thresholds[1:2], pixels_threshold=200, area_threshold=200):
    #     #     left_line = max(l_blob)
    #     # for r_blob in img.find_blobs(self.thresholds[2:3], pixels_threshold=200, area_threshold=200):
    #     #     right_line = max(r_blob)
    #     left_line = self.cam.get_blobs_bottom(self.thresholds[1:2], self.servo.pan_pos)
    #     right_line = self.cam.get_blobs_bottom(self.thresholds[2:3], self.servo.pan_pos)
    #     obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=500, area_threshold=500)
    #     white_blob = img.find_blobs(self.thresholds[4:], pixels_threshold=2000, area_threshold=2000)
    #     return red_line, left_line, right_line, obstacle, white_blob


    def move_servos(self, left_speed, right_speed, duration=200):
        self.servo.set_speed(left_speed, right_speed)
        time.sleep_ms(duration)
        self.servo.set_speed(0, 0)


    def move_offset(self, target_blob_cx, speed=0.1):
        pixel_offset = target_blob_cx - self.cam.w_centre
        steering = self.PID_steering.get_pid(pixel_offset/sensor.width(), scaler=1.0)
        self.drive(speed, steering) # Move pan servo to track block
        # print('steering:', steering, 'offset:', pixel_offset)


    def move_bias(self, target_blob_cx, speed, angle_wanted = 0, turn = 1):
        pixel_error = target_blob_cx - self.cam.w_centre
        pid_error = self.PID.get_pid(pixel_error/sensor.width(),1)
        pan_angle = - turn * (self.servo.pan_pos * turn - angle_wanted + pid_error) / 90
        self.drive(speed, pan_angle)
        time.sleep_ms(50)
        print('pid_error', pid_error, 'pan_angle', turn_speed, 'pan pos', self.servo.pan_pos)


    def pan_angle_decay(self, limit=3, reduce_by=2):
        # decay pan angle after search
        pan_angle = self.servo.pan_pos
        if pan_angle >= limit:
            self.servo.set_angle(pan_angle - reduce_by)
        elif pan_angle <= -limit:
            self.servo.set_angle(pan_angle + reduce_by)
        else:
            self.servo.set_angle(0)
        return


    def stage1(self, speed: float) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        at_end = False
        not_seen_count = 0
        while not at_end:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            target_blob = self.cam.get_biggest_blob(red_line)

            if not target_blob:
                # can't see red line, so search for it
                if not_seen_count < 3:
                    print(f"Can't see red line for {not_seen_count}, but maybe it's a threshold issue.")
                    time.sleep(0.05)
                    not_seen_count += 1
                elif not_seen_count < 6:
                    print(f"Oh no, no target. I'll look for {not_seen_count}")
                    self.servo.set_speed(0, 0)
                    self.scan_for_blob(threshold_idx=0, step=2, limit=40)
                    not_seen_count += 1
                else:
                    print("1. I can't see anything, so I'll stop")
                    at_end = True
            else:
                not_seen_count = 0
                print('Oh yay, the target')
                self.move_offset(target_blob.cx(), speed)
                time.sleep(0.1)
                self.pan_angle_decay()

        self.servo.soft_reset()
        return


    def stage2(self, speed: float) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        at_end = False
        not_seen_count = 0
        while not at_end:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=700, area_threshold=700)
            target_blob = self.cam.get_biggest_blob(red_line)

            if obstacle:
                print('I spy with my little eye, an obstacle')
                # avoid_blob = self.cam.get_biggest_blob(obstacle)
                self.servo.set_speed(0, 0)

            elif not target_blob:
                # can't see red line, so search for it
                if not_seen_count < 3:
                    print(f"Can't see red line for {not_seen_count}, but maybe it's a threshold issue.")
                    time.sleep(0.05)
                    not_seen_count += 1
                elif not_seen_count < 6:
                    print(f"Oh no, no target. I'll look for {not_seen_count}")
                    self.servo.set_speed(0, 0)
                    self.scan_for_blob(threshold_idx=0, step=2, limit=40)
                    not_seen_count += 1
                else:
                    print("1. I can't see anything, so I'll stop")
                    at_end = True

            else:
                not_seen_count = 0
                print('Oh yay, the target')
                self.move_offset(target_blob.cx(), speed)
                time.sleep(0.1)
                self.pan_angle_decay()

        self.servo.soft_reset()
        return


    def get_distance(self, blob):
        y = blob.h()/2 + blob.cy()
        return 6.8282*10**(-9) * y**4 - 6.1122*10**(-6) * y**3 + 0.0018916 * y**2 - 0.31509 * y**1 + 34.7077 * y**0

    def stage3_orig(self, speed: float, stop_me: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        count = 0
        while True:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=700, area_threshold=700)
            target_blob = self.cam.get_biggest_blob(red_line)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            curr_dist = None

            if obstacle:
                print('I spy with my little eye, an obstacle')
                curr_dist = self.get_distance(avoid_blob)
                print(f"The obstacle is {curr_dist} cm away for {avoid_blob.cy()}")

            if curr_dist is not None and curr_dist <= stop_me:
                print("Stopping at:", curr_dist)
                self.servo.set_speed(0, 0)
                break

            elif not target_blob:
                print("Oh no, no target. I'll look!")
                self.servo.set_speed(0, 0)
                self.scan_for_blob(threshold_idx=0, step=2, limit=40)
                count += 1
                if count > 3:
                    break
            else:
                if target_blob:
                    count = 0
                    # img.draw_rectangle(target_blob.rect())  # Debug
                    print('Oh yay, the target')
                    self.move_offset(target_blob.cx(), speed)
                    time.sleep(0.2)
                    # self.move_bias(target_blob.cx(), speed)
                    self.servo.set_angle(0)
                else:
                    print("I can't see anything!")

        self.servo.soft_reset()
        return


    def stage3_new(self, speed: float, stop_me: float) -> None:
        at_end = False
        not_seen_count = 0
        while not at_end:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=700, area_threshold=700)
            target_blob = self.cam.get_biggest_blob(red_line)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            curr_dist = None

            if avoid_blob:
                print('I spy with my little eye, an obstacle')
                curr_dist = self.get_distance(avoid_blob)
                print(f"The obstacle is {curr_dist} cm away for {avoid_blob.cy()}")

            if curr_dist is not None and curr_dist <= stop_me:
                print("Stopping at:", curr_dist, 'cm and breaking.')
                self.servo.set_speed(0, 0)
                break

            elif not target_blob:
                # can't see red line, so search for it
                if not_seen_count < 3:
                    print(f"Can't see red line for {not_seen_count}, but maybe it's a threshold issue.")
                    time.sleep(0.05)
                    not_seen_count += 1
                elif not_seen_count < 6:
                    print(f"Oh no, no target. I'll look for {not_seen_count}")
                    self.servo.set_speed(0, 0)
                    self.scan_for_blob(threshold_idx=0, step=2, limit=40)
                    not_seen_count += 1
                else:
                    print("1. I can't see anything, so I'll stop")
                    at_end = True

            else:
                not_seen_count = 0
                print('Oh yay, the target')
                self.move_offset(target_blob.cx(), speed)
                time.sleep(0.1)
                self.pan_angle_decay()

        self.servo.soft_reset()
        return


    def stage4(self, speed: float, turn_speed: float, stop_me: float, target_angle: float, given_dir: str) -> None:
        at_end = False
        not_seen_count = 0

        while not at_end:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=700, area_threshold=700)
            target_blob = self.cam.get_biggest_blob(red_line)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            curr_dist = None

            if avoid_blob:
                print('I spy with my little eye, an obstacle')
                curr_dist = self.get_distance(avoid_blob)
                print(f"The obstacle is {curr_dist} cm away for {avoid_blob.cy()}")

            if curr_dist is not None and curr_dist <= stop_me:
                print("Stopping at:", curr_dist, 'cm and breaking.')
                self.servo.set_speed(0, 0)
                pan_angle = self.track_blob(avoid_blob)
                if given_dir is None and pan_angle >= 0:
                    print('Object on right, turn left')
                    direction = -1
                elif given_dir is None and pan_angle < 0:
                    print('Object on left, turn right')
                    direction = 1
                else:
                    if given_dir == 'right':  # direction == 1 turns right
                        direction = 1
                        print('Turning', given_dir)
                    elif given_dir == 'left': # direction == -1 turns left
                        direction = -1
                        print('Turning', given_dir)
                break

            elif not target_blob:
                # can't see red line, so search for it
                if not_seen_count < 3:
                    print(f"Can't see red line for {not_seen_count}, but maybe it's a threshold issue.")
                    time.sleep(0.05)
                    not_seen_count += 1
                elif not_seen_count < 6:
                    print(f"Oh no, no target. I'll look for {not_seen_count}")
                    self.servo.set_speed(0, 0)
                    self.scan_for_blob(threshold_idx=0, step=2, limit=40)
                    not_seen_count += 1
                else:
                    print("1. I can't see anything, so I'll stop")
                    at_end = True

            else:
                not_seen_count = 0
                print('Oh yay, the target')
                self.move_offset(target_blob.cx(), speed)
                time.sleep(0.1)
                self.pan_angle_decay()

        # Turn to the target angle
        error = abs(pan_angle) - target_angle # 0 - 30
        not_seen_count = 0

        while abs(pan_angle) < target_angle: # yes, -30, abs(error) > 3 or
            img = sensor.snapshot()

            print(f'Current angle: {pan_angle}, error: {error}, target: {target_angle}')

            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=1000, area_threshold=1000)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            if avoid_blob is None:
                print("Lost obstacle, searching!")
                self.scan_for_blob(threshold_idx=self.obstacle_id, step=2, limit=40)
                not_seen_count += 1
                if not_seen_count > 3:
                    print("Can't find obstacle for", not_seen_count, ", breaking!")
                    break
                continue

            pan_angle = self.track_blob(avoid_blob) # 0
            error = abs(pan_angle) - target_angle # 0 - 30

            self.servo.set_speed(turn_speed * direction, -turn_speed * direction)
            time.sleep_ms(70)
            self.servo.set_speed(0, 0)
            time.sleep_ms(70)

        print(f'Successfully turned to {pan_angle}')
        self.servo.soft_reset()
        return


    def turn_away_from_obs(self, pan_angle, target_angle, turn_speed, direction, t, o_count, max_delay):
        o_count += 1
        print('Obstacle counter', o_count)
        if o_count == 2:
            target_angle += 15
            max_delay = 50
        while abs(pan_angle) < target_angle: # yes, -30
            img = sensor.snapshot()

            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=1000, area_threshold=1000)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            if avoid_blob is None:
                print("Lost obstacle, breaking!")
                break

            pan_angle = self.track_blob(avoid_blob) # 0
            error = abs(pan_angle) - target_angle # 0 - 30
            print(f'Current angle: {pan_angle}, error: {error}, target: {target_angle}')

            self.servo.set_speed(turn_speed * direction, -turn_speed * direction)
            time.sleep_ms(t)
            self.servo.set_speed(0, 0)
            time.sleep_ms(t)

        print(f'Successfully turned to {pan_angle}')
        return pan_angle, o_count, max_delay

    def stage5(self, speed: float, turn_speed: float, stop_me: float, target_angle: float, t:float) -> None:

        at_end = False
        not_seen_count = 0
        lane_id = self.l_line_id
        delay = 0
        max_delay = 25
        o_count = 0
        reached = True
        manual_turns = [-1, 1] # left first, right next

        while not at_end:
            img = sensor.snapshot()

            red_line = img.find_blobs(self.thresholds[:1], pixels_threshold=100, area_threshold=100)
            side_lane = img.find_blobs(self.thresholds[lane_id:lane_id+1], pixels_threshold=100, area_threshold=100)
            obstacle = img.find_blobs(self.thresholds[3:4], pixels_threshold=700, area_threshold=700)
            target_blob = self.cam.get_biggest_blob(red_line)
            lane_blob = self.cam.get_biggest_blob(side_lane)
            avoid_blob = self.cam.get_biggest_blob(obstacle)

            curr_dist = None

            if avoid_blob:
                print('I spy with my little eye, an obstacle')
                curr_dist = self.get_distance(avoid_blob)
                print(f"The obstacle is {curr_dist} cm away for {avoid_blob.cy()}")

            if curr_dist is not None and curr_dist <= stop_me:
                print("Stopping at:", curr_dist, 'cm.')
                self.servo.set_speed(0, 0)
                self.scan_for_blob(self.mid_line_id, step=2, limit=40)
                time.sleep(0.1)

                if target_blob:
                    delay = 0
                    if target_blob.cx() - avoid_blob.cx() > 0:
                        print('Object on left, turn right')
                        direction = -1
                    else:
                        print('Object on right, turn left')
                        direction = 1
                else:
                    pan_angle = self.track_blob(avoid_blob)
                    # if pan_angle >= 0:
                    #     print('Object panned on right, turn left')
                    #     direction = 1
                    # else:
                    #     print('Object panned on left, turn right')
                    #     direction = -1
                    if o_count < 1:
                        print('Manual dir IF')
                        direction = manual_turns[0]
                    else:
                        print('Manual dir ELSE')
                        direction = manual_turns[1]

                pan_angle = self.track_blob(avoid_blob)
                pan_angle, o_count, max_delay = self.turn_away_from_obs(pan_angle, target_angle, turn_speed, direction, t, o_count, max_delay)
                reached = False

                # If object is on the right
                if pan_angle >= 0:
                    print('Object is on the right, taking left lane')
                    lane_id = self.l_line_id
                    delay = max_delay
                else:
                    print('Object is on the left, taking right lane')
                    lane_id = self.r_line_id
                    delay = max_delay
                self.servo.set_angle(0)

            elif target_blob and delay < 1 and reached:
                not_seen_count = 0
                print('Oh yay, the target')
                self.move_offset(target_blob.cx(), speed)
                time.sleep(0.1)
                self.pan_angle_decay()

            elif lane_blob and delay >= 1:
                reached = True
                print('Delay', delay)
                not_seen_count = 0
                delay -= 1
                print('I see', lane_id)
                self.move_offset(lane_blob.cx(), speed)
                time.sleep(0.05)
                self.pan_angle_decay()

            else:
                # can't see any lane, so search for it
                if not_seen_count <= 3:
                    print(f"Can't see red line for {not_seen_count}, but maybe it's a threshold issue.")
                    time.sleep(0.05)
                    not_seen_count += 1
                if not_seen_count <= 6:
                    print(f"Oh no, no target. I'll look for {not_seen_count}")
                    self.servo.set_speed(0, 0)
                    # self.scan_for_all([self.mid_line_id, self.l_line_id, self.r_line_id], step=2, limit=40)
                    self.scan_for_blob(self.mid_line_id, step=2, limit=40)
                    not_seen_count += 1
                else:
                    print("I can't see anything, so I'll stop")
                    at_end = True

        self.servo.soft_reset()
        return


    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


    def track_blob(self, blob) -> None:
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

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)

        return pan_angle


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 30) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        count = 0
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Identify the relevant blob of current color
            img = sensor.snapshot()
            # img.rotation_corr(z_rotation=self.servo.pan_pos)
            blobs = img.find_blobs(self.thresholds, pixels_threshold=200, area_threshold=200)
            colours = self.cam.get_blob_colours(blobs)
            # print('Colours:',colours, 'current blob idx', threshold_idx, 'looking for num:', pow(2, threshold_idx))


            if pow(2, threshold_idx) in colours:
                print('Great Job! I knew I could find it!')
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                print("When scanning",  self.servo.pan_pos )
                count += 1
                self.scan_direction *= -1

            if count > 1:
                print("Didn't find anything :(")
                break

    def scan_for_all(self, threshold_idxs: list, step=2, limit=30) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idxs (list): List of indices along self.cam.thresholds to check for matching blobs.
            step (int): Number of degrees to pan the servo at each scan step.
            limit (int): Scan oscillates between +-limit degrees.
        """
        count = 0
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)
            self.servo.set_angle(new_pan_angle)

            # Capture the image and correct rotation
            img = sensor.snapshot()
            img.rotation_corr(z_rotation=self.servo.pan_pos)
            blobs = img.find_blobs(self.thresholds, pixels_threshold=200, area_threshold=200)
            colours = self.cam.get_blob_colours(blobs)

            # Print which threshold indices we are looking for:
            print('Colours:', colours, 'looking for threshold indices:', threshold_idxs)

            # Iterate over each threshold index and check if its expected value is in colours
            for idx in threshold_idxs:
                if pow(2, idx) in colours:
                    print('Found threshold index:', idx)
                    return idx  # Return the found threshold index

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                count += 1
                self.scan_direction *= -1

            if count > 1:
                print("Didn't find anything :(")
                break

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


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()

if __name__ == "__main__":

    # Threshold order: red_line (0), left_line (2), right_line (4), obstacle (8), white_blob (16)
    thresholds = [
                # (28, 36, 9, 35, -24, 6), # Red 301D gaqin 5
                # (17, 24, 4, 17, -33, -15), # Left
                # (31, 40, -14, -1, -32, -18), # Right
                # (51, 75, -6, 8, 13, 31), # Obstacle 301D gain 5

                (40, 56, 18, 42, -9, 27), # Red (1) 301D gain 12
                (36, 42, -28, -12, -6, 18), # Left (2) Green 301D gain 12
                (34, 45, -16, 6, -34, -9), # Left/Right (2/4) Blue 301D gain 12
                # (17, 30, 5, 20, -20, 0), # Right (4) Purple 301D 12
                (32, 52, -3, 6, 9, 39), # Obstacle (8) 301D gain 12
                # # (65, 86, -8, 10, -18, 6), # White 301D gain 12

                # (27, 45, 19, 38, 8, 27), # Red (0) 27
                # # (37, 63, 13, 37, -27, -10), # Red pranathi, living room
                # (23, 33, -28, -13, 10, 30), # Green Left (2) AA 27
                # (35, 48, -13, 5, -28, -13), # Blue Right (2/4) AA 27
                # # (10, 20, -1, 15, -11, 6), # Purple (4) AA 27
                # (30, 44, -8, 8, 18, 40), # obstacle (8) AA 27
                # # (55, 75, -1, 13, -20, -5), # White (16) AA 27

                # (29, 43, 14, 38, 0, 32), # PP Red
                # (31, 55, -9, 10, -15, 8), ## PP White
                # (1, 2, -3, -2, 0, 3), ## PP obstacle black
                # (4, 40, -12, 15, 5, 36), # PP obstacle
                # (46, 53, -15, -3, -21, -6), # PP blue
                # (14, 22, 0, 14, -15, -4), # PP purple
                ]

    robot = Robot(thresholds, gain=12) # AA gain 29; PP gain 10
    time.sleep(1)
    motor_cam_dist = -3
    motor_front_dist = 0.5

    # robot.stage1(0.1)
    # robot.stage2(0.1)
    # robot.stage3_new(0.08, 10+motor_cam_dist)
    # given = 'right'
    # if given == 'left':
    #     turn = 0.1
    # else:
    #     turn = 0.07
    # robot.stage4(speed=0.08, turn_speed=turn, stop_me=15+motor_cam_dist, target_angle=35, given_dir=given)
    # robot.stage5(speed=0.08, turn_speed=0.08, stop_me=10, target_angle=40, t=70)

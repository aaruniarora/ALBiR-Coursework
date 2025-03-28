from machine import SoftI2C, Pin
from math import asin
import pca9685, time

class Servo:
    """
    A class responsible for controlling servos via the OpenMV board.
    """

    def __init__(self):
        """
        Initialise the servo object and sets the tuning coefficients.
        """
        # Servo tuning coefficients; EDIT these values as required.
        self.pan_angle_corr = 0
        # self.left_zero = 0.05 # original values
        self.left_zero = 0.08
        # self.right_zero = -0.1 # original values
        self.right_zero = -0.07

        # Define servo pin IDs for the servo shield.
        self.pan_id = 1 # original was 7
        self.left_id = 4 # original was 5
        self.right_id = 5 # original was 4

        # Set up servo angle limits
        self.degrees = 90
        self.min_deg = -self.degrees/2
        self.max_deg = self.degrees/2

        self.curr_l_speed = 0
        self.curr_r_speed = 0
        self.pan_pos = 0

        self.freq = 50
        self.period = 1000000 / self.freq

        # Calculate duty cycles for PWM signal range.
        self.min_duty = self._us2duty(700)
        self.max_duty = self._us2duty(2300)
        self.mid_duty = (self.min_duty + self.max_duty) / 2
        self.span = (self.max_duty - self.min_duty)

        # Initialise the PCA9685 controller for I2C communication.
        self.pca9685 = pca9685.PCA9685(SoftI2C(sda=Pin('P5'), scl=Pin('P4')), 0x40)
        self.pca9685.freq(self.freq)


    def set_differential_drive(self, speed: float, bias: float) -> None:
        """
        Set speeds for a differential drive robot using a speed coefficient and a steering bias.

        Args:
            speed_coeff (float): Overall speed coefficient of the robot (0 to 1).
            steering_bias (float): Steering bias for the robot (-1 to 1).
        """
        # Validate input ranges
        speed = max(min(speed, 1), 0)
        bias = max(min(bias, 1), -1)

        # Calculate individual wheel speeds
        left_speed = speed * (1 - bias)
        right_speed = speed * (1 + bias)

        # Normalize speeds if they exceed 1
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1:
            left_speed /= max_speed
            right_speed /= max_speed

        # Set the speeds
        self.set_speed(left_speed, right_speed)


    def set_angle(self, angle: float) -> float:
        """
        Set the pan servo to a specific angle (in degrees).

        Args:
            angle (float): Desired angle (deg) for the camera pan servo.

        Returns:
            float: Corrected angle (deg) of the camera pan servo.
        """
        # Correct for off centre angle
        self.pan_pos = angle

        angle = self.pan_angle_corr + angle

        # Constraint angle to limits
        angle = max(min(angle, self.max_deg), self.min_deg)

        # Compute duty for pca PWM signal
        duty = self.mid_duty + ( self.span * (angle / self.degrees) )

        # Set duty and send PVM signal
        self.pca9685.duty(self.pan_id, int(duty) )

        return self.pan_pos


    def set_speed(self, l_speed: float, r_speed: float) -> None:
        """
        Control the speed of the left and right wheel servos.

        Args:
            l_speed (float): Speed to set left wheel servo to (-1~1).\n
            r_speed (float): Speed to set right wheel servo to (-1~1).
        """

        # Constraint speeds to limits
        l_speed = max(min(l_speed, 1), -1)
        r_speed = max(min(r_speed, 1), -1)
        self.curr_l_speed = l_speed
        self.curr_r_speed = r_speed

        # Convert speed to duty
        l_duty = self.mid_duty + (self.span / 2 * (self.curr_l_speed + self.left_zero))
        # print(f'l_duty: {l_duty}')
        r_duty = self.mid_duty - (self.span / 2 * (self.curr_r_speed + self.right_zero))
        # print(f'r_duty: {r_duty}')

        # Ensure duty cycle values are within the valid range
        l_duty = max(min(l_duty, self.max_duty), self.min_duty)
        r_duty = max(min(r_duty, self.max_duty), self.min_duty)
        # print(f'l_duty: {l_duty} and r_duty: {r_duty}')

        # Set duty and send PWM signal
        self.pca9685.duty(self.left_id, int(l_duty))
        self.pca9685.duty(self.right_id, int(r_duty))

        return

    def _duty2us(self, value: float) -> int:
        """
        Convert a given PWM duty cycle value to microseconds.

        Args:
            value (float): PWM duty cycle value.

        Returns:
            int: Corresponding value in microseconds.
        """
        return int(value * self.period / 4095)


    def _us2duty(self, value: float) -> int:
        """
        Convert a given value in microseconds to PWM duty cycle.

        Args:
            value (float): Value in microseconds.

        Returns:
            int: Corresponding PWM duty cycle value.
        """
        return int(4095 * value / self.period)


    def release(self, idx: int) -> None:
        """
        Simple servo release method

        Args:
            idx (int): Servo shield pin ID to reset.
        """
        self.pca9685.duty(idx, 0)


    def soft_reset(self) -> None:
        """
        Method to reset the servos to default and print a delay prompt.
        """
        # Reset all servo shield pins
        for i in range(0, 7, 1):
            self.pca9685.duty(i, 0)

        # Reset pan to centre
        self.set_angle(0)

        # Print delay prompt
        for i in range(3, 0, -1):
            print(f"{i} seconds remaining.")
            time.sleep_ms(1000)

        print("___Running Code___")


if __name__ == "__main__":
    servo = Servo()
    servo.soft_reset()

    # Servo speed test
    print('\n0,0')
    servo.set_speed(0,0)
    time.sleep_ms(1000)

    print('\n0.1,0.1')
    servo.set_speed(0.1,0.1)
    time.sleep_ms(1000)

    print('\n-0.1, -0.1')
    servo.set_speed(-0.1, -0.1)
    time.sleep_ms(1000)

    # print('\n0.2,0.2')
    # servo.set_speed(0.2,0.2)
    # time.sleep_ms(1000)

    # print('\n-0.2, -0.2')
    # servo.set_speed(-0.2, -0.2)
    # time.sleep_ms(1000)

    # print('\n-30deg')
    # servo.set_speed(-0.01, -0.01)
    # servo.set_angle(-30)
    # time.sleep_ms(1000)

    # print('\n30deg')
    # servo.set_speed(0.01, 0.01)
    # servo.set_angle(30)
    # time.sleep_ms(1000)

    # print('\n-15deg')
    # servo.set_speed(-0.01, -0.01)
    # servo.set_angle(-15)
    # time.sleep_ms(1000)

    # print('\n15deg')
    # # servo.set_speed(0.01, 0.01)
    # servo.set_angle(15)
    # time.sleep_ms(1000)

    servo.soft_reset()

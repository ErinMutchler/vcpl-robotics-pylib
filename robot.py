import pwmio
import digitalio
import rotaryio
import board
import adafruit_hcsr04
import analogio
import time
import math

MOTOR_MIN_PWM = 0
MOTOR_MAX_PWM = (2 ** 16) - 1
MOTOR_MIN_SPEED = 0
MOTOR_MAX_SPEED = 100
MOTOR_TICKS_PER_ROTATION = 1400
MOTOR_CM_PER_ROTATION = 13.5

ROBOT_RADIUS_CM = 4.5


class Motor:
    def __init__(self, pwm_pin, dir_pin, phase_a, phase_b):
        self._pwm = pwmio.PWMOut(pwm_pin, duty_cycle=0, frequency=10000, variable_frequency=False)
        self._dir = digitalio.DigitalInOut(dir_pin)
        self._dir.direction = digitalio.Direction.OUTPUT
        self._encoder = rotaryio.IncrementalEncoder(phase_a, phase_b, divisor=1)

    def on(self, direction, speed):
        if direction.upper() != "FORWARD" and direction.upper() != "BACKWARD":
            raise ValueError("direction must be either 'forward' or 'backward'")

        speed = max(0, min(speed, 100))
        self._dir.value = True if direction.upper() == "FORWARD" else False
        self._pwm.duty_cycle = int(MOTOR_MAX_PWM * speed / 100)

    def off(self):
        self._pwm.duty_cycle = 0

    def reset_encoder_position(self):
        self._encoder.position = 0

    def get_encoder_ticks(self):
        return self._encoder.position

    def get_encoder_centimeters(self):
        return abs(self._encoder.position / MOTOR_TICKS_PER_ROTATION * MOTOR_CM_PER_ROTATION)


class Button:
    def __init__(self, pin):
        self._button = digitalio.DigitalInOut(pin)
        self._button.direction = digitalio.Direction.INPUT
        self._button.pull = digitalio.Pull.DOWN

    def get_state(self):
        return self._button.value


class Robot:
    def __init__(self):
        self.left_motor = Motor(pwm_pin=board.GP13, dir_pin=board.GP12, phase_a=board.GP16, phase_b=board.GP17)
        self.right_motor = Motor(pwm_pin=board.GP14, dir_pin=board.GP15, phase_a=board.GP10, phase_b=board.GP11)

        self._ultrasonic_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.GP9, echo_pin=board.GP8)
        self._left_ir_sensor = analogio.AnalogIn(board.GP27)
        self._right_ir_sensor = analogio.AnalogIn(board.GP28)

        self._left_bumper = Button(board.GP18)
        self._right_bumper = Button(board.GP6)
        self._a_button = Button(board.GP1)
        self._b_button = Button(board.GP0)

    def drive(self, direction, speed):
        self.left_motor.on(direction, speed)
        self.right_motor.on(direction, speed)

    def spin(self, direction, speed):
        self.left_motor.on("BACKWARD" if direction.upper() == "LEFT" else "FORWARD", speed)
        self.right_motor.on("BACKWARD" if direction.upper() == "RIGHT" else "FORWARD", speed)

    def stop(self):
        self.left_motor.off()
        self.right_motor.off()

    def drive_centimeters(self, direction, speed, centimeters):
        kp = 0.4
        ki = 0.0
        kd = 0.001

        error = 0
        prev_error = 0
        integral = 0

        self.left_motor.reset_encoder_position()
        self.right_motor.reset_encoder_position()

        while self.left_motor.get_encoder_centimeters() <= centimeters:
            error = abs(self.left_motor.get_encoder_ticks()) - abs(self.right_motor.get_encoder_ticks())
            correction = (kp * error) + (ki * integral) + (kd * prev_error)

            left_speed = speed - correction
            if left_speed < MOTOR_MIN_PWM or left_speed > MOTOR_MAX_PWM:
                right_speed = speed + correction
            else:
                right_speed = speed

            self.left_motor.on(direction, left_speed)
            self.right_motor.on(direction, right_speed)

            integral += error
            prev_error = error
            print(error)
            time.sleep(0.01)

        self.stop()

    def spin_degrees(self, direction, speed, degrees):
        kp = 0.0
        ki = 0.0
        kd = 0.0

        error = 0
        prev_error = 0
        integral = 0

        centimeters = 2 * math.pi * ROBOT_RADIUS_CM * (degrees / 360)

        self.left_motor.reset_encoder_position()
        self.right_motor.reset_encoder_position()

        while self.left_motor.get_encoder_centimeters() <= centimeters:
            error = abs(self.left_motor.get_encoder_ticks()) - abs(self.right_motor.get_encoder_ticks())
            correction = (kp * error) + (ki * integral) + (kd * prev_error)

            left_speed = speed - correction
            if left_speed == MOTOR_MIN_SPEED or left_speed == MOTOR_MAX_SPEED:
                right_speed = speed + correction
            else:
                right_speed = speed

            self.left_motor.on("BACKWARD" if direction.upper() == "LEFT" else "FORWARD", left_speed)
            self.right_motor.on("BACKWARD" if direction.upper() == "RIGHT" else "FORWARD", right_speed)

            integral += error
            prev_error = error

            time.sleep(0.01)

        self.stop()







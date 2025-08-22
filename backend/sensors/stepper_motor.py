import RPi.GPIO as GPIO
from time import sleep


class StepperMotor:


    def __init__(self, in1, in2, in3, in4):
        
        self.control_pins = [in1, in2, in3, in4]

        for pin in self.control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        self.cw_halfstep_seq = [
          [1,0,0,0],
          [1,1,0,0],
          [0,1,0,0],
          [0,1,1,0],
          [0,0,1,0],
          [0,0,1,1],
          [0,0,0,1],
          [1,0,0,1]
        ]

    def cycle(self, step_speed, reverse=False):
        # 8 halfsteps in one cycle
        for halfstep in range(8):
            # update pin output for each halfstep
            for pin in range(4):
                # spin correct way with cw being forward
                if not reverse:
                    GPIO.output(self.control_pins[pin], self.cw_halfstep_seq[halfstep][pin])
                else:
                    GPIO.output(self.control_pins[pin], self.cw_halfstep_seq[-(halfstep)][pin])
            sleep(step_speed)
             
    def rotate(self, degrees, rotation_speed, reverse=False):
        
        # convert degress to cycles with 512 cycles per 360 degrees
        cycles = int(degrees / 360 * 512)

        # convert RPM to time per step by dividing timer per cycle by 8 steps
        step_speed = (60 / (rotation_speed*512)) / 8

        for i in range(cycles):
            self.cycle(step_speed, reverse)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    motor = StepperMotor(21, 20, 16, 12)

    motor.rotate(360, 30)
    motor.rotate(360, 30, True)

    GPIO.cleanup()

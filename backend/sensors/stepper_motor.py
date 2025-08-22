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
             
    def rotate(self, degrees, rotation_speed, reverse=False, max_accel=0.5):

        # state variables   
        target_speed = rotation_speed
        curr_speed = 0
        curr_accel = 0

        # constants and gains
        MIN_SPEED = 5
        ACCEL_MAX = 15000
        JERK_MAX = 8e5
        dt = 10e-5

        # keep track of how many cycles it took to accelerate
        accel_cycles = 0
        
        # convert degress to cycles with 512 cycles per 360 degrees
        cycles = int(degrees / 360 * 512)

        for i in range(cycles):

            # check for when target speed is reached
            if(curr_speed >= target_speed and accel_cycles == 0):
                accel_cycles = i
                curr_accel = 0

            # standstill speed to avoid pull in stall
            if curr_speed == 0 and target_speed != 0:
                curr_speed = MIN_SPEED

            else:
                # update acceleration and speed
                  
                if i <= cycles - accel_cycles:
                    curr_accel = min(curr_accel+JERK_MAX*dt, ACCEL_MAX) if curr_speed < target_speed else curr_accel
                    curr_speed = min(curr_speed+curr_accel*dt, target_speed)
                else:
                    curr_accel = min(curr_accel+JERK_MAX*dt, ACCEL_MAX)
                    curr_speed = max(curr_speed-curr_accel*dt, MIN_SPEED)                    
           
            # convert RPM to time per step by dividing timer per cycle by 8 steps
            step_speed = (60 / (curr_speed * 512)) / 8

            self.cycle(step_speed, reverse)

            print(f"current speed: {curr_speed}")
            print(f"current accel: {curr_accel}")
            sleep(dt)

           

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    try:
        motor = StepperMotor(21, 20, 16, 12)

        motor.rotate(360, 30)
        motor.rotate(360, 30, True)

    finally:
        GPIO.cleanup()

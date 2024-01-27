import time
import os
import atexit
import logging
import math
from logdecorator import log_on_start, log_on_end, log_on_error

try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command, get_battery_voltage
except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat import reset_mcu, run_command    
    print("Not on PiCar-X. Using simulated robot hat functions")

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# reset robot_hat
reset_mcu()
time.sleep(0.2)

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # --------- config_file ---------
        self.config_file = fileDB(config, 774, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_file.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_file.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_file.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_file.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_file.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_file.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        # Register stop function to be executed at interpreter termination
        atexit.register(self.stop)
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # Linear scaling removed
        # if speed != 0:
        #     speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        
        logging.debug(f"set_motor_speed: {speed}")

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_file.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_file.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

        logging.debug(f"set_dir_servo_angle: {angle_value}")

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_file.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_file.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def get_ackermann_scaling(self):
        l = 0.1
        w = 0.12

        #Calculate inner and outer angles
        inner_angle = math.atan((2*l*math.sin(self.dir_current_angle))/(2*l*math.cos(self.dir_current_angle) - w*math.sin(self.dir_current_angle)))
        outer_angle = math.atan((2*l*math.sin(self.dir_current_angle))/(2*l*math.cos(self.dir_current_angle) + w*math.sin(self.dir_current_angle)))
        
        #If angles are zero set scaling to 1
        if (inner_angle == 0) or (outer_angle == 0):
            scaling = 1
        else:
            abs_outer_angle = abs(outer_angle)
            abs_inner_angle = abs(inner_angle)

            if abs(outer_angle) > abs(inner_angle):
                scaling = abs_inner_angle/abs_outer_angle
            else:
                scaling = abs_outer_angle/abs_inner_angle
        
        return scaling

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX

            # power_scale = (100 - abs_current_angle) / 100.0
            ackermann_scaling = self.get_ackermann_scaling()

            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * ackermann_scaling)
            else:
                self.set_motor_speed(1, -1*speed * ackermann_scaling)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            
            # power_scale = (100 - abs_current_angle) / 100.0
            ackermann_scaling = self.get_ackermann_scaling()
            
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * ackermann_scaling)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * ackermann_scaling)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_file.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_file.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

class Sensing():
    def __init__(self, 
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ):
        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        
    def sense(self) -> list[int]:
        return list.copy(self.grayscale.read())

class Interpreter():
    def __init__(self, 
                sensitivity:int=0.0,
                polarity:bool=0
                ):
        self.sensitivity = sensitivity
        # 1 for white 0 for dark
        self.polarity = polarity
        self.last_turn_factor = 0
        
    def interpret(self, gs_readings: list[int]):
        #Offset correction
        # gs_readings[0] -= 30
        # gs_readings[2] += 100
        logging.debug(f'\nRaw Grayscale Readings: {gs_readings}')

        readings_avg = (sum(gs_readings) / len(gs_readings))
        if(readings_avg == 0):
            readings_avg = 1
        norm_gs_readings = [x/readings_avg for x in gs_readings]
        # logging.debug(f'Normalized Grayscale Readings: {norm_gs_readings}')

        edges = [abs(norm_gs_readings[0] - norm_gs_readings[1]), abs(norm_gs_readings[2] - norm_gs_readings[1])]
        logging.debug(f'Edges: {edges}')

        max_edge = 1.4
        far_right = edges[0] / max_edge
        far_left = edges[1]  / max_edge
        turn_factor = 0

        if(abs(edges[0] - edges[1]) > self.sensitivity):
            if(far_right>far_left):
                #logging.debug(f'TURNING LEFT')
                turn_factor = -constrain(far_right - far_left - self.sensitivity, 0, 1)
                # turn_factor = -constrain(far_right - self.sensitivity, 0, 1)
                # turn_factor = -constrain(far_right, 0, 1)
            elif(far_right<far_left):
                #logging.debug(f'TURNING RIGHT')
                turn_factor = constrain(far_left - far_right - self.sensitivity, 0, 1)
                # turn_factor = constrain(far_left - self.sensitivity, 0, 1)
                # turn_factor = constrain(far_left, 0, 1)
        
        # if(turn_factor == 0):
        #     turn_factor = self.last_turn_factor
        # else:
        #     self.last_turn_factor = turn_factor

        logging.debug(f'TURN FACTOR {turn_factor}')
        return turn_factor

class Controller():
    def __init__(self, picarx: Picarx):
        self.px = picarx

    def follow_line(self, interpreted_sensor_val: int):
        steer_deadzone = 0.1
        steer = False

        if(interpreted_sensor_val > 0):
            if(interpreted_sensor_val>steer_deadzone):
                steer = True
        else:
            if(interpreted_sensor_val<-steer_deadzone):
                steer = True

        # far = 0.9
        # close = 0.40
        
        # low = 0.25
        # mid = 0.5
        # high = 1.0

        # if(interpreted_sensor_val > 0):
        #     if(interpreted_sensor_val>far):
        #         interpreted_sensor_val = high
        #     elif(close<interpreted_sensor_val and interpreted_sensor_val<far):
        #         interpreted_sensor_val = mid
        #     else:
        #         interpreted_sensor_val = low
        # else:
        #     if(interpreted_sensor_val<-far):
        #         interpreted_sensor_val = -high
        #     elif(-close>interpreted_sensor_val and interpreted_sensor_val>-far):
        #         interpreted_sensor_val = -mid
        #     else:
        #         interpreted_sensor_val = -low

        angle_max = px.DIR_MAX
        angle = interpreted_sensor_val*(angle_max)
        angle = constrain(angle, px.DIR_MIN, px.DIR_MAX)

        logging.debug(f'ANGLE: {angle}')
        
        self.px.forward(35)
        if(steer):
            self.px.set_dir_servo_angle(angle)
        time.sleep(0.1)

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensing()
    interpreter = Interpreter()
    controller = Controller(px)

    while True:
        try:
            controller.follow_line(interpreter.interpret(sensor.sense()))
        except KeyboardInterrupt:
            logging.debug(f'Exiting.. {get_battery_voltage()}')
            px.stop()
            break

    px.stop()

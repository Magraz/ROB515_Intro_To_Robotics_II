from picarx_improved import Picarx, constrain
from robot_hat import ADC, Grayscale_Module
from time import sleep
import logging

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
                sensitivity:int=0.1,
                polarity:bool=0
                ):
        self.sensitivity = sensitivity
        # 1 for white 0 for dark
        self.polarity = polarity

        self.last_turn_factor = 0
    
    def interpret_two_led(self, gs_readings: list[int]):

        gs_readings = [gs_readings[0], gs_readings[1]]
        logging.debug(f'\nRaw Grayscale Readings: {gs_readings}')

        readings_avg = (sum(gs_readings) / len(gs_readings))
        if(readings_avg == 0):
            readings_avg = 1
        norm_gs_readings = [x/readings_avg for x in gs_readings]
        # logging.debug(f'Normalized Grayscale Readings: {norm_gs_readings}')

        far_right = norm_gs_readings[0]
        far_left = norm_gs_readings[1]
        turn_factor = 0
        
        if(abs(far_right - far_left) > self.sensitivity):
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
        
    def interpret_three_led(self, gs_readings: list[int], max_edge: float=1.4):
        logging.debug(f'\RAW GRAYSCALE:: {gs_readings}')

        readings_avg = (sum(gs_readings) / len(gs_readings))
        if(readings_avg == 0):
            readings_avg = 1

        norm_gs_readings = [x/readings_avg for x in gs_readings]
        logging.debug(f'Normalized Grayscale Readings: {norm_gs_readings}')

        edges = [abs(norm_gs_readings[0] - norm_gs_readings[1]), abs(norm_gs_readings[2] - norm_gs_readings[1])]
        logging.debug(f'EDGES: {edges}')



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
        
        if(turn_factor == 0):
            turn_factor = self.last_turn_factor
        else:
            self.last_turn_factor = turn_factor

        logging.debug(f'TURN FACTOR: {turn_factor}')
        return turn_factor

class Controller():
    def __init__(self, picarx: Picarx):
        self.px = picarx

    def follow_line(self, interpreted_sensor_val: int, steer_deadzone: float=0.2, steer_delay: float=0.1):
        steer = False

        if(interpreted_sensor_val > 0):
            if(interpreted_sensor_val>steer_deadzone):
                steer = True
        else:
            if(interpreted_sensor_val<-steer_deadzone):
                steer = True

        angle_max = self.px.DIR_MAX + 5
        angle = interpreted_sensor_val*(angle_max)
        angle = constrain(angle, px.DIR_MIN, px.DIR_MAX)

        logging.debug(f'ANGLE: {angle}')
        
        self.px.forward(35)
        if(steer):
            self.px.set_dir_servo_angle(angle)
        sleep(steer_delay)

        # slight = 0.25
        # med = 0.50
        # high = 0.75
        # v_high = 1

        # angle = 0
        # if(interpreted_sensor_val>0):
        #     if 0<interpreted_sensor_val and interpreted_sensor_val<=slight:
        #         angle = px.DIR_MAX*slight
        #     elif slight<interpreted_sensor_val and interpreted_sensor_val<=med:
        #         angle = px.DIR_MAX*med
        #     elif med<interpreted_sensor_val and interpreted_sensor_val<=high:
        #         angle = px.DIR_MAX*high
        #     elif high<interpreted_sensor_val and interpreted_sensor_val<=v_high:
        #         angle = px.DIR_MAX
        # else:
        #     if interpreted_sensor_val>=-slight and 0>interpreted_sensor_val:
        #         angle = -px.DIR_MAX*slight
        #     elif interpreted_sensor_val>=-med and -slight>interpreted_sensor_val:
        #         angle = -px.DIR_MAX*med
        #     elif interpreted_sensor_val>=-high and -med>interpreted_sensor_val:
        #         angle = -px.DIR_MAX*high
        #     elif interpreted_sensor_val>=-v_high and -high>interpreted_sensor_val:
        #         angle = -px.DIR_MAX

        # logging.debug(f'ANGLE: {angle}')
        
        # self.px.forward(30)
        # self.px.set_dir_servo_angle(angle)
        # sleep(steer_delay)

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensing()
    interpreter = Interpreter()
    controller = Controller(px)

    while True:
        try:
            controller.follow_line(interpreter.interpret_three_led(sensor.sense()))
            # controller.follow_line(interpreter.interpret_two_led(sensor.sense()))
        except KeyboardInterrupt:
            logging.debug(f'Exiting..')
            px.stop()
            break

    px.stop()
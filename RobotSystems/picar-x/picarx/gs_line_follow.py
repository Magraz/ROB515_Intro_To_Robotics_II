from picarx_improved import Picarx, constrain
from bus import Bus
from robot_hat import ADC, Grayscale_Module
from time import sleep
import logging
import math

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Sensing():
    def __init__(self, 
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ):
        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        
    def sense(self) -> list[int]:
        return list.copy(self.grayscale.read())
    
    def producer(self, bus: Bus, delay: int = 0):
        while True:
            bus.write(self.sense())
            sleep(delay)


class Interpreter():
    def __init__(self, 
                sensitivity:int=0.0,
                ):
        self.sensitivity = sensitivity
        self.last_turn_factor = 0
        
    def interpret_three_led(self, gs_readings: list[int], max_edge: float=1):
        logging.debug(f'\RAW GRAYSCALE:: {gs_readings}')

        readings_avg = (sum(gs_readings) / len(gs_readings))
        if(readings_avg == 0):
            readings_avg = 1

        norm_gs_readings = [x/readings_avg for x in gs_readings]
        logging.debug(f'Normalized Grayscale Readings: {norm_gs_readings}')

        edges = [abs(1*norm_gs_readings[0] - norm_gs_readings[1]), 1.2*abs(norm_gs_readings[2] - norm_gs_readings[1])]
        logging.debug(f'EDGES: {edges}')

        far_right = (edges[0] / max_edge)
        far_left = (edges[1]  / max_edge)
        turn_factor = 0

        if(abs(edges[0] - edges[1]) > self.sensitivity):
            if(far_right>far_left):
                turn_factor = -constrain(far_right - far_left - self.sensitivity, 0, 1)
            elif(far_right<far_left):
                turn_factor = constrain(far_left - far_right - self.sensitivity, 0, 1)
        
        logging.debug(f'TURN FACTOR: {turn_factor}')

        if math.isclose(turn_factor, 0, abs_tol=0.2):
            turn_factor = self.last_turn_factor
        else:
            self.last_turn_factor = turn_factor

        return turn_factor

    def consumer_producer(self, sense_bus:Bus, interpret_bus:Bus, delay:int):
        while True:
            interpret_bus.write(self.interpret_three_led(gs_readings=sense_bus.read()))
            sleep(delay)
class Controller():
    def __init__(self, picarx: Picarx):
        self.px = picarx

    def follow_line(self, interpreted_sensor_val: int, steer_deadzone: float=0.05, detecting_obstacles: bool=False):
        
        # Check steer deadzone
        steer = False
        if(interpreted_sensor_val > 0):
            if(interpreted_sensor_val>steer_deadzone):
                steer = True
        else:
            if(interpreted_sensor_val<-steer_deadzone):
                steer = True

        # Map turn factor to actual steer angle
        angle_max = self.px.DIR_MAX
        angle = interpreted_sensor_val*(angle_max)
        angle = constrain(angle, px.DIR_MIN, px.DIR_MAX)

        logging.debug(f'ANGLE: {angle}')
        
        #Motion control
        if not detecting_obstacles:
            self.px.forward(35)
        if(steer):
            self.px.set_dir_servo_angle(angle)

    def consumer(self, interpret_bus:Bus, delay:int):
        while True:
            self.follow_line(interpret_bus.read())
            sleep(delay)

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensing()
    interpreter = Interpreter()
    controller = Controller(px)

    while True:
        try:
            controller.follow_line(interpreter.interpret_three_led(sensor.sense()))
        except KeyboardInterrupt:
            logging.debug('Exiting..')
            px.stop()
            break

    px.stop()
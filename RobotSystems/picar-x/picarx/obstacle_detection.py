from picarx_improved import Picarx
import logging

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class Sensing():
    def __init__(self, picarx: Picarx):
        self.px = picarx
        
    def get_distance(self) -> list[int]:
        return self.px.get_distance()


class Interpreter():
    def __init__(self):
        pass
        
    def interpret_distance(self, distance: float):
        print(distance)
        logging.debug(f'RAW ULTRASONIC: {distance}')

        if(distance < 10):
            return True
        else:
            return False

class Controller():
    def __init__(self, picarx: Picarx):
        self.px = picarx

    def move_and_stop(self, obstacle_detected: bool):
        if obstacle_detected:
            self.px.stop()
        else:
            self.px.forward(35)

if __name__ == "__main__":
    px = Picarx()
    sensor = Sensing()
    interpreter = Interpreter()
    controller = Controller(px)

    while True:
        try:
            controller.move_and_stop(interpreter.interpret_distance(sensor.get_distance()))
        except KeyboardInterrupt:
            logging.debug('Exiting..')
            px.stop()
            break

    px.stop()
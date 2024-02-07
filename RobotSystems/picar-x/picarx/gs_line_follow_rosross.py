from picarx_improved import Picarx
from time import sleep
import logging
import sys
import os
import gs_line_follow as lf
import obstacle_detection as od


sys.path.insert(1, os.path.join(sys.path[0], '/home/magraz/ROB515_Intro_To_Robotics_II/RossROS'))

import rossros

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.INFO)

if __name__ == "__main__":
    px = Picarx()

    lf_sensor = lf.Sensing()
    lf_interpreter = lf.Interpreter()
    lf_controller = lf.Controller(px)

    od_sensor = od.Sensing()
    od_interpreter = od.Interpreter()
    od_controller = od.Controller(px)

    runtime_duration= 10

    lf_sensor_delay = 0.01
    lf_interpreter_delay = 0.02
    lf_controller_delay = 0.03

    od_sensor_delay = 0.1
    od_interpreter_delay = 0.1
    od_controller_delay = 0.1

    lf_sensor_bus = rossros.Bus()
    lf_interpreter_bus = rossros.Bus()

    od_sensor_bus = rossros.Bus()
    od_interpreter_bus = rossros.Bus()

    termination_bus = rossros.Bus()

    lf_sensing_p = rossros.Producer(lf_sensor.sense(), lf_sensor_bus, delay = lf_sensor_delay, termination_bus = termination_bus)
    lf_interpreter_cs = rossros.ConsumerProducer(lf_interpreter.interpret_three_led(lf_sensor_bus.get_message()), lf_sensor_bus, lf_interpreter_bus, delay = lf_interpreter_delay, termination_bus = termination_bus)
    lf_controller_c = rossros.Consumer(controller = lf_controller.follow_line(lf_interpreter_bus.get_message(), detecting_obstacles=True), delay = lf_controller_delay, termination_bus = termination_bus)

    od_sensing_p = rossros.Producer(od_sensor.get_distance(), od_sensor_bus, delay = od_sensor_delay, termination_bus = termination_bus)
    od_interpreter_cs = rossros.ConsumerProducer(od_interpreter.interpret_distance(od_sensor_bus.get_message()), od_sensor_bus, od_interpreter_bus, delay = od_interpreter_delay, termination_bus = termination_bus)
    od_controller_c = rossros.Consumer(controller = od_controller.move_and_stop(od_interpreter_bus.get_message()), delay = od_controller_delay, termination_bus = termination_bus)
    
    terminator = rossros.Timer(termination_bus, duration = runtime_duration)
    
    try:
        rossros.runConcurrently([lf_sensing_p, lf_interpreter_cs, lf_controller_c, od_sensing_p, od_interpreter_cs, od_controller_c])

    except KeyboardInterrupt:
        print('Exited...')
        px.stop()

    px.stop()

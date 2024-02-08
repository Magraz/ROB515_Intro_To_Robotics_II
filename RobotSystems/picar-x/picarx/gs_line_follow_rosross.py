from picarx_improved import Picarx
from time import sleep
import logging
import sys
import os
import gs_line_follow as lf
import obstacle_detection as od


sys.path.insert(1, os.path.join(sys.path[0], '/home/magraz-pi/RossROS'))

import rossros

#Set Logging configuration
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

if __name__ == "__main__":
    px = Picarx()

    lf_sensor = lf.Sensing()
    lf_interpreter = lf.Interpreter()
    lf_controller = lf.Controller(px)

    od_sensor = od.Sensing(px)
    od_interpreter = od.Interpreter()
    od_controller = od.Controller(px)

    runtime_duration= 10

    lf_sensor_delay = 0.1
    lf_interpreter_delay = 0.1
    lf_controller_delay = 0.1

    od_sensor_delay = 0.5
    od_interpreter_delay = 0.5
    od_controller_delay = 0.5

    lf_sensor_bus = rossros.Bus(name="lf_sensor_bus")
    lf_interpreter_bus = rossros.Bus(name="lf_interpreter_bus")

    od_sensor_bus = rossros.Bus(name="od_sensor_bus")
    od_interpreter_bus = rossros.Bus(name="od_interpreter_bus")

    termination_bus = rossros.Bus(name="termination_bus")

    lf_sensing_p = rossros.Producer(lf_sensor.sense, lf_sensor_bus, delay = lf_sensor_delay, termination_buses = termination_bus, name="lf_sensing_p")
    lf_interpreter_cs = rossros.ConsumerProducer(lf_interpreter.interpret_three_led, lf_sensor_bus, lf_interpreter_bus, delay = lf_interpreter_delay, termination_buses = termination_bus, name="lf_interpreter_cs")
    lf_controller_c = rossros.Consumer(lf_controller.follow_line, lf_interpreter_bus, delay = lf_controller_delay, termination_buses = termination_bus, name="lf_controller_c")

    od_sensing_p = rossros.Producer(od_sensor.get_distance, od_sensor_bus, delay = od_sensor_delay, termination_buses = termination_bus, name="od_sensing_p")
    od_interpreter_cs = rossros.ConsumerProducer(od_interpreter.interpret_distance, od_sensor_bus, od_interpreter_bus, delay = od_interpreter_delay, termination_buses = termination_bus, name="od_interpreter_cs")
    od_controller_c = rossros.Consumer(od_controller.move_and_stop, od_interpreter_bus,  delay = od_controller_delay, termination_buses = termination_bus, name="od_controller_c")
    
    terminator = rossros.Timer(termination_bus, duration = runtime_duration, name="Termination Timer")
    
    try:
        rossros.runConcurrently([lf_sensing_p, lf_interpreter_cs, lf_controller_c, od_sensing_p, od_interpreter_cs, od_controller_c])
        #rossros.runConcurrently([od_sensing_p, od_interpreter_cs, od_controller_c])
        #rossros.runConcurrently([lf_sensing_p, lf_interpreter_cs, lf_controller_c])

    except KeyboardInterrupt:
        print('Exited...')
        px.stop()

    px.stop()

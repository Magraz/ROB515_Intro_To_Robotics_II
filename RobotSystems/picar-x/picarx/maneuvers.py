from picarx_improved import Picarx
from time import sleep

SPEED = 50

def forward_backward(picarx: Picarx, fw_angl: int, bk_angle: int):
    picarx.set_dir_servo_angle(fw_angl)
    picarx.forward(SPEED)
    sleep(3)
    picarx.set_dir_servo_angle(bk_angle)
    picarx.backward(SPEED)
    sleep(3)
    picarx.set_dir_servo_angle(0)

    px.stop()

def parallel_parking(picarx: Picarx, direction: str):
    angle = 30
    delay = 1.25
    if direction == 'left':
        picarx.set_dir_servo_angle(-angle)
        picarx.backward(SPEED)
        sleep(delay)
        picarx.set_dir_servo_angle(angle)
        picarx.backward(SPEED)
        sleep(delay)
    elif direction == 'right':
        picarx.set_dir_servo_angle(angle)
        picarx.backward(SPEED)
        sleep(delay)
        picarx.set_dir_servo_angle(-angle)
        picarx.backward(SPEED)
        sleep(delay)

    picarx.set_dir_servo_angle(0)

    px.stop()

def three_point_turn(picarx: Picarx, direction: str):
    angle = 30
    delay = 1.5
    return_delay = 2.5

    if direction == 'left':
        picarx.set_dir_servo_angle(-angle)
        picarx.forward(SPEED)
        sleep(delay)

        picarx.set_dir_servo_angle(angle)
        picarx.backward(SPEED)
        sleep(return_delay)
        
    elif direction == 'right':
        picarx.set_dir_servo_angle(angle)
        picarx.forward(SPEED)
        sleep(delay)

        picarx.set_dir_servo_angle(-angle)
        picarx.backward(SPEED)
        sleep(return_delay)

    picarx.set_dir_servo_angle(0)
    picarx.forward(SPEED)
    sleep(delay)

    px.stop()

if __name__ == "__main__":
    
    px = Picarx()

    while True:
        print(
        '''
Available options:
1.- Forward-Backward
2.- Parallel Parking (Right)
3.- Parallel Parking (Left)
4.- 3-Point Turn (Right)
5.- 3-Point Turn (Left)
6.- Exit program
        ''')

        selection = int(input('Select an option [1/2/3/4/5/6]:'))
        
        if selection == 1:
            forward_backward(px, fw_angl=0, bk_angle=20)
        elif selection == 2:
            parallel_parking(px, direction='right')
        elif selection == 3:
            parallel_parking(px, direction='left')
        elif selection == 4:
            three_point_turn(px, direction='right')
        elif selection == 5:
            three_point_turn(px, direction='left')
        elif selection == 6:
            print('\nExiting program...')
            break
        else:
            print('\nInvalid option, tryy')
        
    px.stop()
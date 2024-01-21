# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import logging
import time
import math
import os
import re
from typing import Optional

def set_volume(value):
    """
    Set volume

    :param value: volume(0~100)
    :type value: int
    """
    pass

def run_command(cmd):
    """
    Run command and return status and output

    :param cmd: command to run
    :type cmd: str
    :return: status, output
    :rtype: tuple
    """
    import subprocess
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    result = p.stdout.read().decode('utf-8')
    status = p.poll()
    return status, result

def is_installed(cmd):
    """
    Check if command is installed

    :param cmd: command to check
    :type cmd: str
    :return: True if installed
    :rtype: bool
    """
    status, _ = run_command(f"which {cmd}")
    if status in [0, ]:
        return True
    else:
        return False

def mapping(x, in_min, in_max, out_min, out_max):
    """
    Map value from one range to another range

    :param x: value to map
    :type x: float/int
    :param in_min: input minimum
    :type in_min: float/int
    :param in_max: input maximum
    :type in_max: float/int
    :param out_min: output minimum
    :type out_min: float/int
    :param out_max: output maximum
    :type out_max: float/int
    :return: mapped value
    :rtype: float/int
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_ip(ifaces=['wlan0', 'eth0']):
    """
    Get IP address

    :param ifaces: interfaces to check
    :type ifaces: list
    :return: IP address or False if not found
    :rtype: str/False
    """
    if isinstance(ifaces, str):
        ifaces = [ifaces]
    for iface in list(ifaces):
        search_str = 'ip addr show {}'.format(iface)
        result = os.popen(search_str).read()
        com = re.compile(r'(?<=inet )(.*)(?=\/)', re.M)
        ipv4 = re.search(com, result)
        if ipv4:
            ipv4 = ipv4.groups()[0]
            return ipv4
    return False

class _Basic_class(object):
    """
    Basic Class for all classes

    with debug function
    """
    _class_name = '_Basic_class'
    DEBUG_LEVELS = {'debug': logging.DEBUG,
                    'info': logging.INFO,
                    'warning': logging.WARNING,
                    'error': logging.ERROR,
                    'critical': logging.CRITICAL,
                    }
    """Debug level"""
    DEBUG_NAMES = ['critical', 'error', 'warning', 'info', 'debug']
    """Debug level names"""

    def __init__(self, debug_level='warning'):
        """
        Initialize the basic class

        :param debug_level: debug level, 0(critical), 1(error), 2(warning), 3(info) or 4(debug)
        :type debug_level: str/int
        """
        self.logger = logging.getLogger(f"self._class_name-{time.time()}")
        self.ch = logging.StreamHandler()
        form = "%(asctime)s	[%(levelname)s]	%(message)s"
        self.formatter = logging.Formatter(form)
        self.ch.setFormatter(self.formatter)
        self.logger.addHandler(self.ch)
        self._debug = self.logger.debug
        self._info = self.logger.info
        self._warning = self.logger.warning
        self._error = self.logger.error
        self._critical = self.logger.critical
        self.debug_level = debug_level

    @property
    def debug_level(self):
        """Debug level"""
        return self._debug_level

    @debug_level.setter
    def debug_level(self, debug):
        """Debug level"""
        if debug in range(5):
            self._debug_level = self.DEBUG_NAMES[debug]
        elif debug in self.DEBUG_NAMES:
            self._debug_level = debug
        else:
            raise ValueError(
                f'Debug value must be 0(critical), 1(error), 2(warning), 3(info) or 4(debug), not "{debug}".')
        self.logger.setLevel(self.DEBUG_LEVELS[self._debug_level])
        self.ch.setLevel(self.DEBUG_LEVELS[self._debug_level])
        self._debug(f'Set logging level to [{self._debug_level}]')

def _retry_wrapper(func):
    def wrapper(self, *arg, **kwargs):
        for _ in range(self.RETRY):
            try:
                return func(self, *arg, **kwargs)
            except OSError:
                self._debug(f"OSError: {func.__name__}")
                continue
        else:
            return False
    return wrapper

class I2C(_Basic_class):
    """
    I2C bus read/write functions
    """
    RETRY = 5

    # i2c_lock = multiprocessing.Value('i', 0)

    def __init__(self, address=None, bus=1, *args, **kwargs):
        """
        Initialize the I2C bus

        :param address: I2C device address
        :type address: int
        :param bus: I2C bus number
        :type bus: int
        """
        super().__init__(*args, **kwargs)
        self._bus = bus
        self._smbus = None
        self.address = address

    @_retry_wrapper
    def _write_byte(self, data):   # i2C 写系列函数
        # with I2C.i2c_lock.get_lock():
        self._debug(f"_write_byte: [0x{data:02X}]")
        result = self._smbus.write_byte(self.address, data)
        return result

    @_retry_wrapper
    def _write_byte_data(self, reg, data):
        # with I2C.i2c_lock.get_lock():
        self._debug(f"_write_byte_data: [0x{reg:02X}] [0x{data:02X}]")
        return self._smbus.write_byte_data(self.address, reg, data)

    @_retry_wrapper
    def _write_word_data(self, reg, data):
        # with I2C.i2c_lock.get_lock():
        self._debug(f"_write_word_data: [0x{reg:02X}] [0x{data:04X}]")
        return self._smbus.write_word_data(self.address, reg, data)

    @_retry_wrapper
    def _write_i2c_block_data(self, reg, data):
        # with I2C.i2c_lock.get_lock():
        self._debug(
            f"_write_i2c_block_data: [0x{reg:02X}] {[f'0x{i:02X}' for i in data]}")
        return self._smbus.write_i2c_block_data(self.address, reg, data)

    @_retry_wrapper
    def _read_byte(self):
        # with I2C.i2c_lock.get_lock():
        result = self._smbus.read_byte(self.address)
        self._debug(f"_read_byte: [0x{result:02X}]")
        return result

    @_retry_wrapper
    def _read_byte_data(self, reg):
        # with I2C.i2c_lock.get_lock():
        result = self._smbus.read_byte_data(self.address, reg)
        self._debug(f"_read_byte_data: [0x{reg:02X}] [0x{result:02X}]")
        return result

    @_retry_wrapper
    def _read_word_data(self, reg):
        # with I2C.i2c_lock.get_lock():
        result = self._smbus.read_word_data(self.address, reg)
        result_list = [result & 0xFF, (result >> 8) & 0xFF]
        self._debug(f"_read_word_data: [0x{reg:02X}] [0x{result:04X}]")
        return result_list

    @_retry_wrapper
    def _read_i2c_block_data(self, reg, num):
        # with I2C.i2c_lock.get_lock():
        result = self._smbus.read_i2c_block_data(self.address, reg, num)
        self._debug(
            f"_read_i2c_block_data: [0x{reg:02X}] {[f'0x{i:02X}' for i in result]}")
        return result

    @_retry_wrapper
    def is_ready(self):
        """Check if the I2C device is ready

        :return: True if the I2C device is ready, False otherwise
        :rtype: bool
        """
        addresses = self.scan()
        if self.address in addresses:
            return True
        else:
            return False

    def scan(self):
        """Scan the I2C bus for devices

        :return: List of I2C addresses of devices found
        :rtype: list
        """
        cmd = f"i2cdetect -y {self._bus}"
        # Run the i2cdetect command
        _, output = run_command(cmd)

        # Parse the output
        outputs = output.split('\n')[1:]
        self._debug(f"outputs")
        addresses = []
        for tmp_addresses in outputs:
            if tmp_addresses == "":
                continue
            tmp_addresses = tmp_addresses.split(':')[1]
            # Split the addresses into a list
            tmp_addresses = tmp_addresses.strip().split(' ')
            for address in tmp_addresses:
                if address != '--':
                    addresses.append(int(address, 16))
        self._debug(f"Conneceted i2c device: {addresses}")
        return addresses

    def write(self, data):
        """Write data to the I2C device

        :param data: Data to write
        :type data: int/list/bytearray
        :raises: ValueError if write is not an int, list or bytearray
        """
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, int):
            if data == 0:
                data_all = [0]
            else:
                data_all = []
                while data > 0:
                    data_all.append(data & 0xFF)
                    data >>= 8
        elif isinstance(data, list):
            data_all = data
        else:
            raise ValueError(
                f"write data must be int, list, or bytearray, not {type(data)}")

        # Write data
        if len(data_all) == 1:
            data = data_all[0]
            self._write_byte(data)
        elif len(data_all) == 2:
            reg = data_all[0]
            data = data_all[1]
            self._write_byte_data(reg, data)
        elif len(data_all) == 3:
            reg = data_all[0]
            data = (data_all[2] << 8) + data_all[1]
            self._write_word_data(reg, data)
        else:
            reg = data_all[0]
            data = list(data_all[1:])
            self._write_i2c_block_data(reg, data)

    def read(self, length=1):
        """Read data from I2C device

        :param length: Number of bytes to receive
        :type length: int
        :return: Received data
        :rtype: list
        """
        if not isinstance(length, int):
            raise ValueError(f"length must be int, not {type(length)}")

        result = []
        for _ in range(length):
            result.append(self._read_byte())
        return result

    def mem_write(self, data, memaddr):
        """Send data to specific register address

        :param data: Data to send, int, list or bytearray
        :type data: int/list/bytearray
        :param memaddr: Register address
        :type memaddr: int
        :raise ValueError: If data is not int, list, or bytearray
        """
        if isinstance(data, bytearray):
            data_all = list(data)
        elif isinstance(data, list):
            data_all = data
        elif isinstance(data, int):
            data_all = []
            while data > 0:
                data_all.append(data & 0xFF)
                data >>= 8
        else:
            raise ValueError(
                "memery write require arguement of bytearray, list, int less than 0xFF")
        self._write_i2c_block_data(memaddr, data_all)

    def mem_read(self, length, memaddr):
        """Read data from specific register address

        :param length: Number of bytes to receive
        :type length: int
        :param memaddr: Register address
        :type memaddr: int
        :return: Received bytearray data or False if error
        :rtype: list/False
        """
        result = self._read_i2c_block_data(memaddr, length)
        return result

    def is_avaliable(self):
        """
        Check if the I2C device is avaliable

        :return: True if the I2C device is avaliable, False otherwise
        :rtype: bool
        """
        return self.address in self.scan()

class Pin(_Basic_class):
    """Pin manipulation class"""

    OUT = 0
    """Pin mode output"""
    IN = 1
    """Pin mode input"""
    IRQ_FALLING = 2
    """Pin interrupt falling"""
    IRQ_RISING = 1
    """Pin interrupt falling"""
    IRQ_RISING_FALLING = 3
    """Pin interrupt both rising and falling"""
    PULL_UP = 2
    """Pin internal pull up"""
    PULL_DOWN = 1
    """Pin internal pull down"""
    PULL_NONE = None
    """Pin internal pull none"""

    _dict = {
        "D0":  17,
        "D1":   4,  # Changed
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,  # Removed
        "D7":   4,  # Removed
        "D8":   5,  # Removed
        "D9":   6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  25,  # Changed
        "USER": 25,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST":  5,  # Changed
        "CE": 8,
    }

    def __init__(self, pin, mode=None, pull=None, *args, **kwargs):
        """
        Initialize a pin

        :param pin: pin number of Raspberry Pi
        :type pin: int/str
        :param mode: pin mode(IN/OUT)
        :type mode: int
        :param pull: pin pull up/down(PUD_UP/PUD_DOWN/PUD_NONE)
        :type pull: int
        """
        pass

    def setup(self, mode, pull=None):
        """
        Setup the pin

        :param mode: pin mode(IN/OUT)
        :type mode: int
        :param pull: pin pull up/down(PUD_UP/PUD_DOWN/PUD_NONE)
        :type pull: int
        """
        pass

    def dict(self, _dict=None):
        """
        Set/get the pin dictionary

        :param _dict: pin dictionary, leave it empty to get the dictionary
        :type _dict: dict
        :return: pin dictionary
        :rtype: dict
        """
        if _dict is None:
            return self._dict
        else:
            if not isinstance(_dict, dict):
                raise ValueError(
                    f'Argument should be a pin dictionary like {{"my pin": ezblock.Pin.cpu.GPIO17}}, not {_dict}')
            self._dict = _dict

    def __call__(self, value):
        """
        Set/get the pin value

        :param value: pin value, leave it empty to get the value(0/1)
        :type value: int
        :return: pin value(0/1)
        :rtype: int
        """
        return self.value(value)

    def value(self, value=None):
        """
        Set/get the pin value

        :param value: pin value, leave it empty to get the value(0/1)
        :type value: int
        :return: pin value(0/1)
        :rtype: int
        """
        if value is None:
            result = 0
            return result
        else:
            return value

    def on(self):
        """
        Set pin on(high)

        :return: pin value(1)
        :rtype: int
        """
        return self.value(1)

    def off(self):
        """
        Set pin off(low)

        :return: pin value(0)
        :rtype: int
        """
        return self.value(0)

    def high(self):
        """
        Set pin high(1)

        :return: pin value(1)
        :rtype: int
        """
        return self.on()

    def low(self):
        """
        Set pin low(0)

        :return: pin value(0)
        :rtype: int
        """
        return self.off()

    def irq(self, handler=None, trigger=None, bouncetime=200):
        """
        Set the pin interrupt

        :param handler: interrupt handler callback function
        :type handler: function
        :param trigger: interrupt trigger(RISING, FALLING, RISING_FALLING)
        :type trigger: int
        :param bouncetime: interrupt bouncetime in miliseconds
        :type bouncetime: int
        """
        pass

    def name(self):
        """
        Get the pin name

        :return: pin name
        :rtype: str
        """
        return f"GPIO{self._pin}"

def reset_mcu():
    """
    Reset mcu on Robot Hat.

    This is helpful if the mcu somehow stuck in a I2C data
    transfer loop, and Raspberry Pi getting IOError while
    Reading ADC, manipulating PWM, etc.
    """
    mcu_reset = Pin("MCURST")
    mcu_reset.off()
    time.sleep(0.01)
    mcu_reset.on()
    time.sleep(0.01)

class ADC(I2C):
    """
    Analog to digital converter
    """
    ADDR = 0x14

    def __init__(self, chn, *args, **kwargs):
        """
        Analog to digital converter

        :param chn: channel number (0-7/A0-A7)
        :type chn: int/str
        """
        super().__init__(self.ADDR, *args, **kwargs)
        if isinstance(chn, str):
            # If chn is a string, assume it's a pin name, remove A and convert to int
            if chn.startswith("A"):
                chn = int(chn[1:])
            else:
                raise ValueError(
                    f'ADC channel should be between [A0, A7], not "{chn}"')
        # Make sure channel is between 0 and 7
        if chn < 0 or chn > 7:
            raise ValueError(
                f'ADC channel should be between [0, 7], not "{chn}"')
        chn = 7 - chn
        # Convert to Register value
        self.chn = chn | 0x10

    def read(self):
        """
        Read the ADC value

        :return: ADC value(0-4095)
        :rtype: int
        """
        # Write register address
        self.write([self.chn, 0, 0])
        # Read values
        msb, lsb = super().read(2)

        # Combine MSB and LSB
        value = (msb << 8) + lsb
        self._debug(f"Read value: {value}")
        return value

    def read_voltage(self):
        """
        Read the ADC value and convert to voltage

        :return: Voltage value(0-3.3(V))
        :rtype: float
        """
        # Read ADC value
        value = self.read()
        # Convert to voltage
        voltage = value * 3.3 / 4095
        self._debug(f"Read voltage: {voltage}")
        return voltage

def get_battery_voltage():
    """
    Get battery voltage

    :return: battery voltage(V)
    :rtype: float
    """
    adc = ADC("A4")
    raw_voltage = adc.read_voltage()
    voltage = raw_voltage * 3
    return voltage

timer = [{"arr": 1}] * 4

class PWM(I2C):
    """Pulse width modulation (PWM)"""

    REG_CHN = 0x20
    """Channel register prefix"""
    REG_PSC = 0x40
    """Prescaler register prefix"""
    REG_ARR = 0x44
    """Period registor prefix"""

    ADDR = 0x14

    CLOCK = 72000000.0
    """Clock frequency"""

    def __init__(self, channel, *args, **kwargs):
        """
        Initialize PWM

        :param channel: PWM channel number(0-13/P0-P13)
        :type channel: int/str
        """
        super().__init__(self.ADDR, *args, **kwargs)
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError(
                    f'PWM channel should be between [P0, P13], not "{channel}"')
        if isinstance(channel, int):
            if channel > 13 or channel < 0:
                raise ValueError(
                    f'channel must be in range of 0-13, not "{channel}"')

        self.channel = channel
        self.timer = int(channel/4)
        self._pulse_width = 0
        self._freq = 50
        self.freq(50)

    def _i2c_write(self, reg, value):
        pass

    def freq(self, freq=None):
        """
        Set/get frequency, leave blank to get frequency

        :param freq: frequency(0-65535)(Hz)
        :type freq: float
        :return: frequency
        :rtype: float
        """
        if freq is None:
            return self._freq

        self._freq = int(freq)
        # [prescaler,arr] list
        result_ap = []
        # accuracy list
        result_acy = []
        # middle value for equal arr prescaler
        st = int(math.sqrt(self.CLOCK/self._freq))
        # get -5 value as start
        st -= 5
        # prevent negetive value
        if st <= 0:
            st = 1
        for psc in range(st, st+10):
            arr = int(self.CLOCK/self._freq/psc)
            result_ap.append([psc, arr])
            result_acy.append(abs(self._freq-self.CLOCK/psc/arr))
        i = result_acy.index(min(result_acy))
        psc = result_ap[i][0]
        arr = result_ap[i][1]
        self._debug(f"prescaler: {psc}, period: {arr}")
        self.prescaler(psc)
        self.period(arr)

    def prescaler(self, prescaler=None):
        """
        Set/get prescaler, leave blank to get prescaler

        :param prescaler: prescaler(0-65535)
        :type prescaler: int
        :return: prescaler
        :rtype: int
        """
        if prescaler is None:
            return self._prescaler

        self._prescaler = round(prescaler)
        self._freq = self.CLOCK/self._prescaler/timer[self.timer]["arr"]
        reg = self.REG_PSC + self.timer
        self._debug(f"Set prescaler to: {self._prescaler}")
        self._i2c_write(reg, self._prescaler-1)

    def period(self, arr=None):
        """
        Set/get period, leave blank to get period

        :param arr: period(0-65535)
        :type arr: int
        :return: period
        :rtype: int
        """
        global timer
        if arr is None:
            return timer[self.timer]["arr"]

        timer[self.timer]["arr"] = round(arr)
        self._freq = self.CLOCK/self._prescaler/timer[self.timer]["arr"]
        reg = self.REG_ARR + self.timer
        self._debug(f"Set arr to: {timer[self.timer]['arr']}")
        self._i2c_write(reg, timer[self.timer]["arr"])

    def pulse_width(self, pulse_width=None):
        """
        Set/get pulse width, leave blank to get pulse width

        :param pulse_width: pulse width(0-65535)
        :type pulse_width: float
        :return: pulse width
        :rtype: float
        """
        if pulse_width is None:
            return self._pulse_width

        self._pulse_width = int(pulse_width)
        reg = self.REG_CHN + self.channel
        self._i2c_write(reg, self._pulse_width)

    def pulse_width_percent(self, pulse_width_percent=None):
        """
        Set/get pulse width percentage, leave blank to get pulse width percentage

        :param pulse_width_percent: pulse width percentage(0-100)
        :type pulse_width_percent: float
        :return: pulse width percentage
        :rtype: float
        """
        global timer
        if pulse_width_percent is None:
            return self._pulse_width_percent

        self._pulse_width_percent = pulse_width_percent
        temp = self._pulse_width_percent / 100.0
        # print(temp)
        pulse_width = temp * timer[self.timer]["arr"]
        self.pulse_width(pulse_width)

def test():
    import time
    p = PWM(0)
    # p.debug = 'debug'
    p.period(1000)
    p.prescaler(10)
    # p.pulse_width(2048)
    while True:
        for i in range(0, 4095, 10):
            p.pulse_width(i)
            print(i)
            time.sleep(1/4095)
        time.sleep(1)
        for i in range(4095, 0, -10):
            p.pulse_width(i)
            print(i)
            time.sleep(1/4095)
        time.sleep(1)

def test2():
    p = PWM("P0")
    while True:
        p.pulse_width_percent(50)

class Servo(PWM):
    """Servo motor class"""
    MAX_PW = 2500
    MIN_PW = 500
    FREQ = 50
    PERIOD = 4095

    def __init__(self, channel, *args, **kwargs):
        """
        Initialize the servo motor class

        :param channel: PWM channel number(0-14/P0-P14)
        :type channel: int/str
        """
        super().__init__(channel, *args, **kwargs)
        self.period(self.PERIOD)
        prescaler = self.CLOCK / self.FREQ / self.PERIOD
        self.prescaler(prescaler)

    def angle(self, angle):
        """
        Set the angle of the servo motor

        :param angle: angle(-90~90)
        :type angle: float
        """
        if not (isinstance(angle, int) or isinstance(angle, float)):
            raise ValueError(
                "Angle value should be int or float value, not %s" % type(angle))
        if angle < -90:
            angle = -90
        if angle > 90:
            angle = 90
        self._debug(f"Set angle to: {angle}")
        pulse_width_time = mapping(angle, -90, 90, self.MIN_PW, self.MAX_PW)
        self._debug(f"Pulse width: {pulse_width_time}")
        self.pulse_width_time(pulse_width_time)

    def pulse_width_time(self, pulse_width_time):
        """
        Set the pulse width of the servo motor

        :param pulse_width_time: pulse width time(500~2500)
        :type pulse_width_time: float
        """
        if pulse_width_time > self.MAX_PW:
            pulse_width_time = self.MAX_PW
        if pulse_width_time < self.MIN_PW:
            pulse_width_time = self.MIN_PW

        pwr = pulse_width_time / 20000
        self._debug(f"pulse width rate: {pwr}")
        value = int(pwr * self.PERIOD)
        self._debug(f"pulse width value: {value}")
        self.pulse_width(value)

class fileDB(object):
	"""A file based database.

    A file based database, read and write arguements in the specific file.
    """
	def __init__(self, db:str, mode:str=None, owner:str=None):  
		'''
		Init the db_file is a file to save the datas.
		
		:param db: the file to save the datas.
		:type db: str
		:param mode: the mode of the file.
		:type mode: str
		:param owner: the owner of the file.
		:type owner: str
		'''

		pass

	def file_check_create(self, file_path:str, mode:str=None, owner:str=None):
		"""
		Check if file is existed, otherwise create one.
		
		:param file_path: the file to check
		:type file_path: str
		:param mode: the mode of the file.
		:type mode: str
		:param owner: the owner of the file.
		:type owner: str
		"""
		pass
	
	def get(self, name, default_value=None):
		"""
		Get value with data's name
		
		:param name: the name of the arguement
		:type name: str
		:param default_value: the default value of the arguement
		:type default_value: str
		:return: the value of the arguement
		:rtype: str
		"""
		return default_value
	
	def set(self, name, value):
		"""
		Set value by with name. Or create one if the arguement does not exist
		
		:param name: the name of the arguement
		:type name: str
		:param value: the value of the arguement
		:type value: str
		"""
		pass

class Ultrasonic():
    """UltraSonic modules"""

    def __init__(self, trig: Pin, echo: Pin, timeout: float = 0.02):
        """
        Initialize the ultrasonic class

        :param trig: trig pin
        :type trig: robot_hat.Pin
        :param echo: echo pin
        :type echo: robot_hat.Pin
        :param timeout: timeout in seconds
        :type timeout: float
        :raise ValueError: if trig or echo is not a Pin object
        """
        if not isinstance(trig, Pin):
            raise TypeError("trig must be robot_hat.Pin object")
        if not isinstance(echo, Pin):
            raise TypeError("echo must be robot_hat.Pin object")
        self.trig = trig
        self.echo = echo
        self.timeout = timeout

    def _read(self) -> float:
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        if cm > 1000:
            return -1
        return cm

    def read(self, times: Optional[int] = 10) -> float:
        """
        Read distance in cm

        :param times: times try to read
        :type times: int
        :return: distance in cm, -1 if timeout
        :rtype: float
        """
        for _ in range(times):
            a = self._read()
            if a != -1:
                return a
        return -1
    
class Grayscale_Module(object):
    """3 channel Grayscale Module"""

    LEFT = 0
    """Left Channel"""
    MIDDLE = 1
    """Middle Channel"""
    RIGHT = 2
    """Right Channel"""

    REFERENCE_DEFAULT = [1000]*3

    def __init__(self, pin0: ADC, pin1: ADC, pin2: ADC, reference: int = None):
        """
        Initialize Grayscale Module

        :param pin0: ADC object or int for channel 0
        :type pin0: robot_hat.ADC/int
        :param pin1: ADC object or int for channel 1
        :type pin1: robot_hat.ADC/int
        :param pin2: ADC object or int for channel 2
        :type pin2: robot_hat.ADC/int
        :param reference: reference voltage
        :type reference: 1*3 list, [int, int, int]
        """
        self.pins = (pin0, pin1, pin2)
        for i, pin in enumerate(self.pins):
            if not isinstance(pin, ADC):
                raise TypeError(f"pin{i} must be robot_hat.ADC")
        self._reference = self.REFERENCE_DEFAULT

    def reference(self, ref: list = None) -> list:
        """
        Get Set reference value

        :param ref: reference value, None to get reference value
        :type ref: list
        :return: reference value
        :rtype: list
        """
        if ref is not None:
            if isinstance(ref, list) and len(ref) == 3:
                self._reference = ref
            else:
                raise TypeError("ref parameter must be 1*3 list.")
        return self._reference

    def read_status(self, datas: list = None) -> list:
        """
        Read line status

        :param datas: list of grayscale datas, if None, read from sensor
        :type datas: list
        :return: list of line status, 0 for white, 1 for black
        :rtype: list
        """
        if self._reference is None:
            raise ValueError("Reference value is not set")
        if datas is None:
            datas = self.read()
        return [0 if data > self._reference[i] else 1 for i, data in enumerate(datas)]

    def read(self, channel: int = None) -> list:
        """
        read a channel or all datas

        :param channel: channel to read, leave empty to read all. 0, 1, 2 or Grayscale_Module.LEFT, Grayscale_Module.CENTER, Grayscale_Module.RIGHT 
        :type channel: int/None
        :return: list of grayscale data
        :rtype: list
        """
        if channel is None:
            return [self.pins[i].read() for i in range(3)]
        else:
            return self.pins[channel].read()


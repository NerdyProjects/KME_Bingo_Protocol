import os
from enum import IntEnum, Enum
import struct
import time
import socket

def val_to_rpm(val):
    return 15000000 / val

def rpm_to_val(rpm):
    return round(15000000 / rpm)

def rpm_to_16bit_val(rpm):
    val = rpm_to_val(rpm)
    return [val & 0x0FF, (val & 0x0FF00) >> 8]

# 0x5e 94 is 10 degrees
# 0x63 99 is 12 degrees
# 0x68 104 is 14 degrees
def engine_temp_to_val(temp):
    return round(temp * 2.5 + 70)

def val_to_engine_temp(val):
    return (val - 70) / 2.5
#50 is 1.0V
# ADC range 0-255 0..5V
def val_to_voltage(val):
    return (val / 255) * 5.1

def voltage_to_val(voltage):
    return int((voltage / 5.1) * 255)

def seconds_to_lambda_delay(seconds):
    return round(seconds / 5)

def lambda_delay_to_seconds(v):
    return v * 5

class LevelSensorType(Enum):
    PW12 = 0
    RESERVE = 1
    NINETY_OHMS = 2

class LambdaType(Enum):
    V0_1 = 0
    V0_5N = 1
    V0_5P = 2
    V5_0N = 3
    V5_0P = 4
    V08_16 = 5

class TPSType(Enum):
    LIN_0_5 = 0
    LIN_5_0 = 1
    NOT_INSTALLED = 2
    SW_0_12 = 6
    SW_12_0 = 7
    BOSCH_MONO = 8


class LambdaEmulation(Enum):
    SIGNAL = 0
    GROUND = 1
    DISCONNECTED = 2

class DrivingMode(Enum):
    NORMAL = 0
    ECO = 1
    SPORT = 2

class RegisterAddress(IntEnum):
    VERSION = 0x01
    STATUS = 0x02
    SETTINGS_1 = 0x03
    SETTINGS_2 = 0x04
    OPTION_1 = 0x06
    LAMBDA_TPS = 0x07
    LAMBDA_NEUTRAL = 0x08
    LAMBDA_DELAY = 0x09
    OPTION_2 = 0x0A
    ENGINE_TEMP_MIN = 0x10
    IAP = 0x13
    RPM_TOP_LIMIT_L = 0x1C
    RPM_TOP_LIMIT_H = 0x1D
    EMERGENCY_LPG_TIME = 0x26


class Memory:
    require_minimum_engine_temperature = False
    use_learned_iap = True
    stepper_jump_to_iap = False
    lpg_switch_when_rpm_increases = True
    cutoff_enabled = False
    rpm_top_limit_enabled = False
    level_sensor_type = LevelSensorType.RESERVE
    lambda_emulation_type = LambdaEmulation.DISCONNECTED
    rpm_signal_low_level = False
    allow_emergency_engine_start = True
    driving_mode = DrivingMode.NORMAL
    control_panel_has_gas_level_indication = False

    registers = {
            RegisterAddress.VERSION : 'get_version',
            RegisterAddress.STATUS: 'get_status',
            RegisterAddress.SETTINGS_1: 'get_settings_1',
            RegisterAddress.SETTINGS_2: 'get_settings_2',
            RegisterAddress.OPTION_1: 'set_option',
            RegisterAddress.OPTION_2: 'set_emulation_type'
            }

    memory = {
            RegisterAddress.LAMBDA_TPS: 0,
            RegisterAddress.LAMBDA_NEUTRAL: 0,
            RegisterAddress.LAMBDA_DELAY: 1,
            RegisterAddress.ENGINE_TEMP_MIN: 0,
            RegisterAddress.IAP: 32,
            RegisterAddress.RPM_TOP_LIMIT_L: 0x73,
            RegisterAddress.RPM_TOP_LIMIT_H: 0x09,
            RegisterAddress.EMERGENCY_LPG_TIME: 0,
            }

    def get_version(self, v):
        return [0x4b, # unknown
            0x63 # upper nibble lower bytes describe major version, lower nibble describes minor version: 4.03
            ]

    def get_settings_2(self, v):
        return [0x28,
                0x28,
                0xc8,
                0x1e,
                0x70,
                0x17,
                0x06,
                0xd4,
                0x30,
                0x1e,
                0xc4,
                0x09,
                0xf5,
                0x32
                ]

    lambda_delay = seconds_to_lambda_delay(10)

    tps_status = voltage_to_val(1.5)
    lambda_status = voltage_to_val(0.7)
    stepper_position = 80
    rpm = 800

    def setRegister(self, reg, v):
        if reg in self.registers:
            return getattr(self, self.registers[reg])(v)

        if reg in self.memory:
            self.memory[reg] = v
            return [v]
        else:
            print("Write to unknown memory address ", reg, ": ", v)
            return [0]

    def get_option(self):
        option = 0
        if self.require_minimum_engine_temperature:
            option += 2
        if self.use_learned_iap:
            option += 4
        if self.stepper_jump_to_iap:
            option += 8
        if self.lpg_switch_when_rpm_increases:
            option += 16
        if self.cutoff_enabled:
            option += 32
        if self.rpm_top_limit_enabled:
            option += 64
        if self.level_sensor_type == LevelSensorType.RESERVE:
            option += 1
        elif self.level_sensor_type == LevelSensorType.NINETY_OHMS:
            option += 128

        return option

    def set_option(self, option):
        self.require_minimum_engine_temperature = bool(option & 2)
        self.use_learned_iap = bool(option & 4)
        self.stepper_jump_to_iap = bool(option & 8)
        self.lpg_switch_when_rpm_increases = bool(option & 16)
        self.cutoff_enabled = bool(option & 32)
        self.rpm_top_limit_enabled = bool(option & 64)
        level_sensor = ((option & 128) >> 7) | (option & 1)
        self.level_sensor_type = LevelSensorType(level_sensor)
        return [self.get_option()]

    def get_emulation_type(self):
        option = self.lambda_emulation_type.value |\
        int(self.rpm_signal_low_level) << 2 |\
        int(self.allow_emergency_engine_start) << 3 |\
        self.driving_mode.value << 4 |\
        self.control_panel_has_gas_level_indication << 7
        return option

    def set_emulation_type(self, emulationType):
        self.lambda_emulation_type = LambdaEmulation(emulationType & 0x03)
        self.rpm_signal_low_level = bool(emulationType & 0x04)
        self.allow_emergency_engine_start = bool(emulationType & 0x08)
        self.driving_mode = DrivingMode((emulationType & 0x30) >> 4)
        self.control_panel_has_gas_level_indication = bool(emulationType & 0x80)
        if emulationType & 0x40:
            print('received set bit in unknown emulation type byte')
        return [self.get_emulation_type()]

    def get_settings_1(self, v):
        res =  [self.get_option(),
                self.memory[RegisterAddress.LAMBDA_TPS],
                self.memory[RegisterAddress.LAMBDA_NEUTRAL],
                self.memory[RegisterAddress.LAMBDA_DELAY],
                self.get_emulation_type(),
                0x0B,
                0x0B,
                self.memory[RegisterAddress.EMERGENCY_LPG_TIME],
                0x18,
                0x0A,
                self.memory[RegisterAddress.ENGINE_TEMP_MIN],
                0x05
                ]
        return res

    def get_status(self, v):
        res = [
                self.tps_status, # TPS voltage in ADC steps 0..5V
                self.lambda_status, # lambda voltage in ADC steps 0..5V
                self.stepper_position, # stepper position 0..256
                self.memory[RegisterAddress.IAP],
                *rpm_to_16bit_val(self.rpm), # 2 byte rpm
                0x8c, # bit 4/5/6/7: TPS field 1-4 bit 6: bit 3: Ignition state bit 0/1/2: toggles lambda sensor color (yellow on 0,2,3, green on 1/7)
                0x10,
                0x01
               ]
        return res

    def getLambdaType(self):
        return LambdaType(self.memory[RegisterAddress.LAMBDA_TPS] >> 4)

    def getTpsType(self):
        return TPSType(self.memory[RegisterAddress.LAMBDA_TPS] & 0x0F)

    def setLambdaType(self, lambda_type):
        self.memory[RegisterAddress.LAMBDA_TPS] = (self.memory[RegisterAddress.LAMBDA_TPS] & 0x0F) | (lambda_type.value << 4)

    def setTPSType(self, tps_type):
        self.memory[RegisterAddress.LAMBDA_TPS] = (self.memory[RegisterAddress.LAMBDA_TPS] & 0xF0) | (tps_type.value & 0x0F)

    def setMinimumEngineTemperature(self, temperature):
        # original software allows 0x5e to 0xd0
        # Range not tested!
        self.memory[RegisterAddress.ENGINE_TEMP_MIN] = engine_temp_to_val(temperature)

    def getMinimumEngineTemperature(self):
        return val_to_engine_temp(self.memory[RegisterAddress.ENGINE_TEMP_MIN])

    def setLambdaNeutralVoltage(self, voltage):
        self.memory[RegisterAddress.LAMBDA_NEUTRAL] = voltage_to_val(voltage)

    def getLambdaNeutralVoltage(self):
        return val_to_voltage(self.memory[RegisterAddress.LAMBDA_NEUTRAL])

    def setEmergencyLpgTime(self, time):
        # original software allows 0.1-10.0 seconds)
        self.memory[RegisterAddress.EMERGENCY_LPG_TIME] = round(time * 10)

    def getEmergencyLpgTime(self):
        return self.memory[RegisterAddress.EMERGENCY_LPG_TIME] / 10

    def getIap(self):
        return self.memory[RegisterAddress.IAP]

    def setIap(self, iap):
        self.memory[RegisterAddress.IAP] = iap

    def getLambdaDelay(self):
        return self.memory[RegisterAddress.LAMBDA_DELAY] * 5

    def setLambdaDelay(self, delay):
        self.memory[RegisterAddress.LAMBDA_DELAY] = round(delay / 5)


class ParserState(Enum):
    WAIT_START = 1
    READ_REG = 2
    READ_VAL = 3
    READ_CHECKSUM = 4

def checksum(arr):
    s = sum(arr)
    return s % 256

def create_response(data):
    res = [0x65]
    res.extend(data)
    res.append(checksum(res))
    return bytes(res)

def handle_data(register, value, memory):
    return create_response(memory.setRegister(register, value))

#m, s = os.openpty()
#print('Slave terminal: ', os.ttyname(s))

#m = os.open('/tmp/virser', os.O_RDWR)
#os.set_blocking(m, False)
client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM + socket.SOCK_NONBLOCK)
client.connect("/tmp/virser")

mode = ParserState.WAIT_START
mem = Memory()

while(True):
    c = None
    try:
        #c = os.read(m, 1)
        c = client.recv(1)
    except Exception:
        pass
    if c is None or len(c) < 1:
        time.sleep(0.1)
        continue
    v = c[0]
    if mode is ParserState.WAIT_START:
        if v == 0x65:
            mode = ParserState.READ_REG
    elif mode is ParserState.READ_REG:
        reg = v
        mode = ParserState.READ_VAL
    elif mode is ParserState.READ_VAL:
        val = v
        mode = ParserState.READ_CHECKSUM
    elif mode is ParserState.READ_CHECKSUM:
        if checksum([0x65, reg, val]) == v:
            data = handle_data(reg, val, mem)
            #written_bytes = os.write(m, data)
            written_bytes = client.send(data)
            if written_bytes != len(data):
                print('Attempting to write ', data, ' only wrote first ', written_bytes)
        else:
            print('checksum error on ', hex(reg), hex(val))
        mode = ParserState.WAIT_START




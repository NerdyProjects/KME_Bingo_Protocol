import os
from enum import IntEnum, Enum
import struct
import time
import socket

def val_to_rpm(val):
    return 15000000 / val

def rpm_to_val(rpm):
    if rpm > 0:
        return round(15000000 / rpm)
    else:
        return 65535


def rpm_to_16bit_val(rpm):
    val = rpm_to_val(rpm)
    return (val & 0x0FF, (val & 0x0FF00) >> 8)

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


class IgnitionType(Enum):
    CYL1_COIL1 = 1
    CYL2_COIL1 = 2
    CYL2_COIL2 = 3
    CYL3_COIL1 = 4
    CYL3_COIL3 = 5
    CYL4_COIL1 = 6
    CYL4_COIL2 = 7
    CYL4_COIL4 = 8
    CYL5_COIL1 = 9
    CYL5_COIL5 = 10
    CYL6_COIL1 = 11
    CYL6_COIL2 = 12
    CYL6_COIL3 = 13
    CYL6_COIL6 = 14
    CYL8_COIL1 = 15
    CYL8_COIL2 = 16
    CYL8_COIL4 = 17
    CYL8_COIL8 = 18


class RegisterAddress(IntEnum):
    VERSION = 0x01
    STATUS = 0x02
    SETTINGS_1 = 0x03
    SETTINGS_2 = 0x04
    EXTENDED_VERSION = 0x05
    OPTION_1 = 0x06
    LAMBDA_TPS = 0x07
    LAMBDA_NEUTRAL = 0x08
    LAMBDA_DELAY = 0x09
    OPTION_2 = 0x0A
    LAMBDA_EMULATION_HIGH_TIME = 0x0B
    LAMBDA_EMULATION_LOW_TIME = 0x0C
    STEPPER_IDLE_SPEED_CORRECTION = 0x0D
    STEPPER_LOAD_SPEED_CORRECTION = 0x0E
    LPG_PETROL_OVERLAP_TIME = 0x0F
    ENGINE_TEMP_MIN = 0x10
    STEPPER_MAX_LOAD_POS = 0x11
    STEPPER_MIN_LOAD_POS = 0x12
    IAP = 0x13
    TPS_JUMP_VOLTAGE = 0x14 # 0.0V: 0, 0.1V: 5, 0.2V: 10, 5.0V: 0xFA
    TPS_JUMP_ENRICHMENT = 0x15
    RPM_SWITCH_TO_LPG_L = 0x16
    RPM_SWITCH_TO_LPG_H = 0x17
    IGNITION_TYPE = 0x18
    CUTOFF_RPM_L = 0x19
    CUTOFF_RPM_H = 0x1A
    CUTOFF_MIXTURE_LEANING = 0x1B
    RPM_TOP_LIMIT_L = 0x1C
    RPM_TOP_LIMIT_H = 0x1D
    TPS_IDLE_VOLTAGE = 0x1E
    UNKNOWN_MEM_1 = 0x1F
    STEPPER_MAX_IDLE_POS = 0x20
    STEPPER_MIN_IDLE_POS = 0x21
    LEVEL_SENSOR_LEVEL1 = 0x22
    LEVEL_SENSOR_LEVEL2 = 0x23
    LEVEL_SENSOR_LEVEL3 = 0x24
    LEVEL_SENSOR_LEVEL4 = 0x25

    EMERGENCY_LPG_TIME = 0x26
    # erase working time on LPG writes 0 to 0x27, 0x28, 0x32
    FIRST_USE_MONTH_DAY = 0x29   #0x1e is 31. january, 0x20 is 1. feb, 0x0F is 16th of jan.
    FIRST_USE_YEAR = 0x2A        #0 is 2000
    # writing registration number 1 writes 0x31 to 0x2b, 0 to 2c-31
    # Reset writes 0x32 to 0x00 (and then 0x08?)
    # Reset writes to 0x1F: 0x33
    # learn writes to 0x1F: 0x32
    UNKNOWN_MEM_2 = 0x32


class Memory:
    # settings I was too lazy to put in setters/getters/memory model yet
    require_minimum_engine_temperature = False
    use_learned_iap = True
    enrich_above_tps_val = False
    lpg_switch_when_rpm_increases = True
    cutoff_enabled = False
    rpm_top_limit_enabled = False
    level_sensor_type = LevelSensorType.NINETY_OHMS
    lambda_emulation_type = LambdaEmulation.DISCONNECTED
    rpm_signal_low_level = False
    allow_emergency_engine_start = True
    driving_mode = DrivingMode.NORMAL
    control_panel_has_gas_level_indication = False

    # device status
    tps_voltage = voltage_to_val(0.14)
    lambda_voltage = voltage_to_val(0.3)
    stepper_position = 80
    rpm = 0
    ignition = True
    tps_state = 0 # 0-4 to mark detected TPS range: Cutoff/Idle/load/full load?
    lambda_state = 1 # 0-3 to mark lambda yellow/green/yellow/red

    registers = {
            RegisterAddress.VERSION : 'get_version',
            RegisterAddress.STATUS: 'get_status',
            RegisterAddress.SETTINGS_1: 'get_settings_1',
            RegisterAddress.SETTINGS_2: 'get_settings_2',
            RegisterAddress.OPTION_1: 'set_option',
            RegisterAddress.OPTION_2: 'set_emulation_type',
            RegisterAddress.EXTENDED_VERSION: 'get_extended_version'
            }

    memory = {
            RegisterAddress.LAMBDA_TPS: 0,
            RegisterAddress.LAMBDA_NEUTRAL: 0,
            RegisterAddress.LAMBDA_DELAY: 1,
            RegisterAddress.ENGINE_TEMP_MIN: 0,
            RegisterAddress.IAP: 32,
            RegisterAddress.RPM_SWITCH_TO_LPG_L: 0x70,
            RegisterAddress.RPM_SWITCH_TO_LPG_H: 0x17,
            RegisterAddress.RPM_TOP_LIMIT_L: 0x73,
            RegisterAddress.RPM_TOP_LIMIT_H: 0x09,
            RegisterAddress.EMERGENCY_LPG_TIME: 0,
            RegisterAddress.LAMBDA_EMULATION_HIGH_TIME: 11,
            RegisterAddress.LAMBDA_EMULATION_LOW_TIME: 11,
            RegisterAddress.IGNITION_TYPE: IgnitionType.CYL4_COIL1.value,
            RegisterAddress.TPS_IDLE_VOLTAGE: 7,
            RegisterAddress.LPG_PETROL_OVERLAP_TIME : 10,
            RegisterAddress.CUTOFF_MIXTURE_LEANING : 30,
            RegisterAddress.CUTOFF_RPM_L: 0xD4,
            RegisterAddress.CUTOFF_RPM_H: 0x30,
            RegisterAddress.TPS_JUMP_VOLTAGE: 0x90,
            RegisterAddress.TPS_JUMP_ENRICHMENT: 0x30,
            RegisterAddress.STEPPER_MAX_LOAD_POS: 10,
            RegisterAddress.STEPPER_MIN_LOAD_POS: 11,
            RegisterAddress.STEPPER_MAX_IDLE_POS: 12,
            RegisterAddress.STEPPER_MIN_IDLE_POS: 13,
            RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION: 0xA9,
            RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION: 0x75,
            RegisterAddress.LEVEL_SENSOR_LEVEL1: 0x10,
            RegisterAddress.LEVEL_SENSOR_LEVEL2: 0x20,
            RegisterAddress.LEVEL_SENSOR_LEVEL3: 0x30,
            RegisterAddress.LEVEL_SENSOR_LEVEL4: 0x40,
            RegisterAddress.UNKNOWN_MEM_1: 0x33,
            RegisterAddress.UNKNOWN_MEM_2: 0x08,
            }

    def get_version(self, v):
        return [0x4b, # unknown
            0x63 # upper nibble lower bits describe major version, lower nibble describes minor version: 4.03
            ]

    def get_settings_2(self, v):
        return [self.memory[RegisterAddress.STEPPER_MAX_LOAD_POS],
                self.memory[RegisterAddress.STEPPER_MIN_LOAD_POS],
                self.memory[RegisterAddress.TPS_JUMP_VOLTAGE],
                self.memory[RegisterAddress.TPS_JUMP_ENRICHMENT],
                self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_L],
                self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_H],
                self.memory[RegisterAddress.IGNITION_TYPE],
                self.memory[RegisterAddress.CUTOFF_RPM_L],
                self.memory[RegisterAddress.CUTOFF_RPM_H],
                self.memory[RegisterAddress.CUTOFF_MIXTURE_LEANING],
                self.memory[RegisterAddress.RPM_TOP_LIMIT_L],
                self.memory[RegisterAddress.RPM_TOP_LIMIT_H],
                self.memory[RegisterAddress.STEPPER_MAX_IDLE_POS],
                self.memory[RegisterAddress.STEPPER_MIN_IDLE_POS],
                ]


    def setRegister(self, reg, v):
        if reg in self.registers:
            return getattr(self, self.registers[reg])(v)

        if reg in self.memory:
            old = self.memory[reg]
            self.memory[reg] = v
            print('write ', hex(reg), ' := ', hex(v), ' (prev: ', hex(old), ')')
            return [v]
        else:
            print("Write to unknown memory address ", hex(reg), ": ", hex(v))
            return [0]

    def get_option(self):
        option = 0
        if self.require_minimum_engine_temperature:
            option += 2
        if self.use_learned_iap:
            option += 4
        if self.enrich_above_tps_val:
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
        self.enrich_above_tps_val = bool(option & 8)
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
                self.memory[RegisterAddress.LAMBDA_EMULATION_HIGH_TIME],
                self.memory[RegisterAddress.LAMBDA_EMULATION_LOW_TIME],
                self.memory[RegisterAddress.EMERGENCY_LPG_TIME],
                0x10,
                self.memory[RegisterAddress.LPG_PETROL_OVERLAP_TIME],
                self.memory[RegisterAddress.ENGINE_TEMP_MIN],
                self.memory[RegisterAddress.TPS_IDLE_VOLTAGE],
                ]
        return res

    lpg_off_high_rpm = False
    cutoff = False
    lpg = True
    engine_temp = 0x60  # 0x45 is 0 degrees, 0xff is 110 degrees, increments don't seem linear

    def get_status(self, v):
        lambda_tps_state = 0
        if self.tps_state and self.tps_state <= 4:
            lambda_tps_state = (1 << (self.tps_state - 1)) << 4
        if self.lambda_state and self.lambda_state <= 3:
            lambda_tps_state |= (1 << (self.lambda_state - 1))

        res = [
                self.tps_voltage, # TPS voltage in ADC steps 0..5V
                self.lambda_voltage, # lambda voltage in ADC steps 0..5V
                self.stepper_position, # stepper position 0..256
                self.memory[RegisterAddress.IAP],
                *rpm_to_16bit_val(self.rpm), # 2 byte rpm
                lambda_tps_state | int(self.ignition) << 3 | self.lambda_state, # bit 4/5/6/7: TPS field 1-4 bit 6: bit 3: Ignition state bit 0/1/2: toggles lambda sensor color (yellow on 0,2,3, green on 1/7)
                0x10 | int(self.cutoff) << 3 | int(self.lpg_off_high_rpm) << 2 | int(self.lpg), # unknown flags: 7, 6, 5, 4, 1
                self.engine_temp
               ]
        return res

    def get_extended_version(self, v):
        return [0x01,
                0x02,
                0x03,
                0x04,
                0x05,
                self.memory[RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION],
                self.memory[RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION],
                0x08,
                0x09,
                0x10,
                0x20,
                0x30,
                0x40,
                0x50,
                0x60,
                0x70,
                0x80,
                0x90,
                0xb0]



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

    def setLambdaEmulationTiming(self, high, low):
        self.memory[RegisterAddress.LAMBDA_EMULATION_HIGH_TIME] = round(high * 40)
        self.memory[RegisterAddress.LAMBDA_EMULATION_LOW_TIME] = round(low * 40)

    def getLambdaEmulationTiming(self):
        return (self.memory[RegisterAddress.LAMBDA_EMULATION_HIGH_TIME] / 40,
                self.memory[RegisterAddress.LAMBDA_EMULATION_LOW_TIME] / 40)

    def setRpmTopLimit(self, rpm):
        self.memory[RegisterAddress.RPM_TOP_LIMIT_L], self.memory[RegisterAddress.RPM_TOP_LIMIT_H] = rpm_to_16bit_val(rpm)

    def getRpmTopLimit(self):
        return val_to_rpm(self.memory[RegisterAddress.RPM_TOP_LIMIT_L] + self.memory[RegisterAddress.RPM_TOP_LIMIT_H] * 256)

    def setRpmSwitchToLPG(self, rpm):
        self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_L], self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_H] = rpm_to_16bit_val(rpm)

    def getRpmTopLimit(self):
        return val_to_rpm(self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_L] + self.memory[RegisterAddress.RPM_SWITCH_TO_LPG_H] * 256)

    def getTpsIdleVoltage(self):
        return val_to_voltage(self.memory[RegisterAddress.TPS_IDLE_VOLTAGE])

    def setTpsIdleVoltage(self, voltage):
        # scaling might be different here: 0.04V is 1, 0.06V is 2, ...
        self.memory[RegisterAddress.TPS_IDLE_VOLTAGE] = voltage_to_val(voltage)

    def getLpgPetrolOverlapTime(self):
        return self.memory[RegisterAddress.LPG_PETROL_OVERLAP_TIME] / 10

    def setLpgPetrolOverlapTime(self, seconds):
        self.memory[RegisterAddress.LPG_PETROL_OVERLAP_TIME] = round(seconds * 10)

    def setCutoffMixtureLeaningFactor(self, factor):
        # original range: 2-100, says it is relatively to IAP
        self.memory[RegisterAddress.CUTOFF_MIXTURE_LEANING] = round(factor * 100)

    def getCutoffMixtureLeaningFactor(self):
        return self.memory[RegisterAddress.CUTOFF_MIXTURE_LEANING] / 100

    def setCutoffRpm(self, rpm):
        self.memory[RegisterAddress.CUTOFF_RPM_L], self.memory[RegisterAddress.CUTOFF_RPM_H] = rpm_to_16bit_val(rpm)

    def getRpmTopLimit(self):
        return val_to_rpm(self.memory[RegisterAddress.CUTOFF_RPM_L] + self.memory[RegisterAddress.CUTOFF_RPM_H] * 256)

    def setIdleOpeningSpeedCorrection(self, val):
        # range: -12 .. +18 in steps of 2.
        if val >= -12 and val <= 18:
            self.memory[RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION] = (self.memory[RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION] & 0xF0) | round((18 - val) / 2)

    def setIdleClosingSpeedCorrection(self, val):
        # range: -10 .. +20 in steps of 2.
        if val >= -10 and val <= 20:
            self.memory[RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION] = (self.memory[RegisterAddress.STEPPER_IDLE_SPEED_CORRECTION] & 0x0F) | (round((20 - val) / 2) << 4)

    def setLoadOpeningSpeedCorrection(self, val):
        # range: -20 .. +10 in steps of 2.
        if val >= -20 and val <= 10:
            self.memory[RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION] = (self.memory[RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION] & 0xF0) | round((10 - val) / 2)

    def setLoadClosingSpeedCorrection(self, val):
        # range: -16 .. +14 in steps of 2.
        if val >= -16 and val <= 14:
            self.memory[RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION] = (self.memory[RegisterAddress.STEPPER_LOAD_SPEED_CORRECTION] & 0x0F) | (round((14 - val) / 2) << 4)


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




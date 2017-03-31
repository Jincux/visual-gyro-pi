import smbus
import time
import math

bus = smbus.SMBus(1)
address = 0x68

mag_address = 0x0C

GYRO_FULL_SCALE_250_DPS   = 0x00
GYRO_FULL_SCALE_500_DPS   = 0x08
GYRO_FULL_SCALE_1000_DPS  = 0x10
GYRO_FULL_SCALE_2000_DPS  = 0x18

ACC_FULL_SCALE_2_G        = 0x00
ACC_FULL_SCALE_4_G        = 0x08
ACC_FULL_SCALE_8_G        = 0x10
ACC_FULL_SCALE_16_G       = 0x18

CALIBRATION               = [0] * 6

def init():
    bus.write_byte_data(address, 27,   GYRO_FULL_SCALE_2000_DPS)
    bus.write_byte_data(address, 28,   ACC_FULL_SCALE_16_G)
    bus.write_byte_data(address, 0x37, 0x02)

    bus.write_byte_data(address, 0x0A, 0x01)

    CALIBRATION = loadCalibData()

def loadCalibData():
    ret = [0, 1] * 3

    data = open('./calib.dat', 'rb').read()
    axes = data.split('\n')
    for axis in axes:
        field = axis.split('\t')

        name = field[0]
        if name == 'x':
            idx = 0
        else if name == 'y':
            idx = 2
        else if name == 'z':
            idx = 4

        ret[idx]   = field[1] # offset
        ret[idx+1] = field[2] # scale

    return ret


def raw_to_int(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)

def poll_data_range(adr, start, length):
    ret = [None] * length;
    for i in range(0, length):
        ret[i] = bus.read_byte_data(adr, start+i)

    return ret


def tick(prnt):
    raw = poll_data_range(address, 0x3B, 14)

    ax = raw_to_int(raw[0], raw[1])
    ay = raw_to_int(raw[2], raw[3])
    az = raw_to_int(raw[4], raw[5])

    # change units to G's
    ax *= (16.0/32768.0)
    ay *= (16.0/32768.0)
    az *= (16.0/32768.0)


    # offset and scale data according to calibration
    ax += CALIBRATION[0]
    ax *= CALIBRATION[1]
    ay += CALIBRATION[2]
    ay *= CALIBRATION[3]
    az += CALIBRATION[4]
    az *= CALIBRATION[5]

    # should equal 1 when under no acceleration
    amag = math.sqrt(math.pow(ax, 2) + math.pow(ay, 2) + math.pow(az, 2))

    gx = raw_to_int(raw[8], raw[9])
    gy = raw_to_int(raw[10], raw[11])
    gz = raw_to_int(raw[12], raw[13])

    gx *= (2000.0/32768.0)
    gy *= (2000.0/32768.0)
    gz *= (2000.0/32768.0)

    if prnt:
        print ""
        print "ax: %s g" % ax
        print "ay: %s g" % ay
        print "az: %s g" % az
        print "am: %s g" % amag

        print "gx: %s deg/s" % gx
        print "gy: %s deg/s" % gy
        print "gz: %s deg/s" % gz

# Execution

init()
while True:
    tick(1)
    time.sleep(0.1)

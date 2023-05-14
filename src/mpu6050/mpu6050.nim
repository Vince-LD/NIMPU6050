import std/[math, endians, sugar, sequtils]
import nesper
import nesper/esp-idf-lib/i2cdev_utils
import nesper/esp-idf-lib/i2cdev
import nesper/timers

export i2cdev_utils

import mpu6050_consts as C

const
    TAG = "MPU6050"

    DEFAULT_SAMPLE_RATE* = 0x20

    ACCEL_RANGE* = [2, 4, 8, 16]
    GYRO_RANGE* = [250, 500, 1000, 2000]


type 
    # mpu6050_addr_t = uint8
    i2c_dev_ptr = ptr i2c_dev_t

    Fields = enum ACCX = 0, ACCY, ACCZ, TEMP, GYROX, GYROY, GYROZ
    MPU6050Readings*[T] = array[Fields, T]

    MPU6050Device* = ref object
        # dev_addr*: mpu6050_addr_t
        device*: i2c_dev_t
        device_ptr*: i2c_dev_ptr
        rate*: uint8
        buffer*: array[16, byte]
        sensors*: array[14, byte]
        gyro_range*: int
        accel_range*: int


    MPU6050Config* = ref object
        dev*: MPU6050Device



proc identify*(mpu6050: MPU6050Device) =
    var who: uint8
    var res = i2c_read_reg(mpu6050.device_ptr, C.MPU6050_RA_WHO_AM_I, who)
    if who != C.MPU6050_ADDRESS_AD0_LOW:
        TAG.logd("Could not answer MPU6050 'Who am I' 0x%02x", C.MPU6050_RA_WHO_AM_I)
    if res != ESP_OK:
        TAG.logd("Could not read from register 0x%02x", res)

proc writeByte*(mpu6050: MPU6050Device, reg: byte, val: byte)=
    mpu6050.buffer[0] = val
    var res = i2c_write_reg(mpu6050.device_ptr, reg, val)
    if res != ESP_OK:
        TAG.logd("Could not write from register 0x%02x", res)

proc readByte*(mpu6050: MPU6050Device, reg: byte): byte =
    var res = i2c_dev_read_reg(
        mpu6050.device_ptr, 
        reg,
        addr mpu6050.buffer,
        1)
    if res != ESP_OK:
        TAG.logd("Could not read from register 0x%02x", res)
    return mpu6050.buffer[0]

proc setBitfield*(mpu6050: MPU6050Device, reg, pos, length, val: byte) =
    let 
        old = readByte(mpu6050, reg)
        shift = pos - length + 1
        mask = byte(2 ^ length - 1) shl shift
        new_val = (old and not mask) or (val shl shift)
    mpu6050.writeByte(reg, new_val)

proc readWord*(mpu6050: MPU6050Device, reg: byte): uint16 = 
    let res = i2c_dev_read_reg(
        mpu6050.device_ptr, 
        reg,
        addr mpu6050.buffer,
        2)
    if res != ESP_OK:
        TAG.logd("Could not read from register 0x%02x", res)
    var word: uint16
    bigEndian16(addr word, addr mpu6050.buffer)
    return word 

proc reset*(mpu6050: MPU6050Device) =
    mpu6050.writeByte(
        C.MPU6050_RA_PWR_MGMT_1, byte((1 shl C.MPU6050_PWR1_DEVICE_RESET_BIT))
    )
    delay(100.Millis)

    mpu6050.write_byte(
            C.MPU6050_RA_SIGNAL_PATH_RESET,
            byte(
                (1 shl C.MPU6050_PATHRESET_GYRO_RESET_BIT) or
                (1 shl C.MPU6050_PATHRESET_ACCEL_RESET_BIT) or
                (1 shl C.MPU6050_PATHRESET_TEMP_RESET_BIT) 
            ),
        )
    delay(100.Millis)

proc setGyroRange*(mpu6050: MPU6050Device, fsr: byte) =
        mpu6050.gyro_range = GYRO_RANGE[fsr]
        mpu6050.setBitfield(
            C.MPU6050_RA_GYRO_CONFIG,
            C.MPU6050_GCONFIG_FS_SEL_BIT,
            C.MPU6050_GCONFIG_FS_SEL_LENGTH,
            fsr,
        )

proc setAccelRange*(mpu6050: MPU6050Device, fsr: byte) =
        mpu6050.accel_range = ACCEL_RANGE[fsr]
        mpu6050.setBitfield(
            C.MPU6050_RA_ACCEL_CONFIG,
            C.MPU6050_ACONFIG_AFS_SEL_BIT,
            C.MPU6050_ACONFIG_AFS_SEL_LENGTH,
            fsr,
        )

proc unpackReadingsArr*[T](rawReadingBuffer: var array[14, byte]): MPU6050Readings[T] =
    var 
        readings_i: MPU6050Readings[int]
        readings_t: MPU6050Readings[T]

    for idx, _ in readings_i:
        bigEndian16(addr readings_i[idx], addr rawReadingBuffer[ord(idx) * 2])
    readings_t[0..6] = readings_i.map((v) => v.T)
    return readings_t

proc readSensorsBytes*(mpu6050: MPU6050Device): array[14, byte] =
    var res = i2c_dev_read_reg(
        mpu6050.device_ptr, 
        C.MPU6050_RA_ACCEL_XOUT_H,
        addr mpu6050.sensors,
        mpu6050.sensors.len().csize_t)
    if res != ESP_OK:
        TAG.logd("Could not read from register 0x%02x", res)
    return mpu6050.sensors

proc readSensors*[T](mpu6050: MPU6050Device): MPU6050Readings[T] =
    discard mpu6050.readSensorsBytes()
    return unpackReadingsArr[T](mpu6050.sensors)

proc readSensorsScaled*(mpu6050: MPU6050Device): MPU6050Readings[float] =
    discard mpu6050.readSensorsBytes()
    var floatReadings = unpackReadingsArr[float](mpu6050.sensors)

    floatReadings[0..2] = floatReadings[0..2].map((x) => x / float(65536 div mpu6050.accel_range div 2))
    floatReadings[4..6] = floatReadings[0..2].map((x) => x / float(65536 div mpu6050.gyro_range div 2)) 

    return floatReadings


proc setDhpfMode*(mpu6050: MPU6050Device, bandwidth: byte) =
        mpu6050.setBitfield(
            C.MPU6050_RA_ACCEL_CONFIG,
            C.MPU6050_ACONFIG_ACCEL_HPF_BIT,
            C.MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
            bandwidth,
        )

proc getDhpfMode*(mpu6050: MPU6050Device): byte =
    return mpu6050.readByte(C.MPU6050_RA_ACCEL_CONFIG)

proc setMotionDetectionThreshold*(mpu6050: MPU6050Device, threshold: byte) =
    mpu6050.writeByte(C.MPU6050_RA_MOT_THR, threshold)

proc setMotionDetectionDuration*(mpu6050: MPU6050Device, duration: byte) =
    mpu6050.writeByte(C.MPU6050_RA_MOT_DUR, duration)

proc enableMotionInterrupt*(mpu6050: MPU6050Device) =
    mpu6050.setBitfield(C.MPU6050_RA_INT_ENABLE, C.MPU6050_INTERRUPT_MOT_BIT, 1, 1)

proc disableMotionInterrupt*(mpu6050: MPU6050Device) =
    # This is a docstring
    mpu6050.setBitfield(C.MPU6050_RA_INT_ENABLE, C.MPU6050_INTERRUPT_MOT_BIT, 1, 0)

proc getSensorAvg*[T](mpu6050: MPU6050Device, samples: int, softstart: int=100): MPU6050Readings[T] =
    var sample = mpu6050.readSensors[T]()
    var cumsumReadings: MPU6050Readings[T]

    for i in countup(0, samples + softstart):
        # the sleep here is to ensure we read a new sample
        # each time
        delay(2.Millis)

        discard mpu6050.readSensors()
        if i < softstart:
            continue

        for j, val in mpu6050.sensors:
            cumsumReadings[j] =  cumsumReadings[j] + val
    
    var avgReadings: MPU6050Readings[T]
    for k, _ in avgReadings:
        avgReadings[k] = T(cumsumReadings[k] / samples)
    
    return avgReadings


proc initDevice*(mpu6050: MPU6050Device) =
    mpu6050.identify()

    # disable sleep mode and select clock source
    mpu6050.writeByte(C.MPU6050_RA_PWR_MGMT_1, C.MPU6050_CLOCK_PLL_XGYRO)

    # enable all sensors
    mpu6050.writeByte(C.MPU6050_RA_PWR_MGMT_2, 0)

    # set sampling rate
    mpu6050.writeByte(C.MPU6050_RA_SMPLRT_DIV, mpu6050.rate)

    # enable dlpf
    mpu6050.writeByte(C.MPU6050_RA_CONFIG, 1)

    # explicitly set accel/gyro range
    mpu6050.setAccelRange(C.MPU6050_ACCEL_FS_2)
    mpu6050.setGyroRange(C.MPU6050_GYRO_FS_250)

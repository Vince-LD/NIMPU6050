import ../src/mpu6050

import options
import nesper
import nesper/timers



let
    i2cCfg = I2CDevConfig(port: I2C_NUM_0, sda: GPIO_NUM_23, scl: GPIO_NUM_22)

var 
    mpu = MPU6050Device(
        rate: DEFAULT_SAMPLE_RATE,
        gyro_range: GYRO_RANGE[0],
        accel_range: ACCEL_RANGE[0],
    )

mpu.initDevice()

app_main():
    while true:
        echo mpu.readSensorsScaled()
        delay(250.Millis)

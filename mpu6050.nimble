# Package

version       = "0.1.0"
author        = "Vincent Le Du"
description   = "Simple library for the I2C MPU6050 accelerometer library. Developped on esp32 but probably usable with other boards."
license       = "MIT"
srcDir        = "src"


# Dependencies

requires "nim >= 1.6.12"
requires "nesper"
# includes nimble tasks for building Nim esp-idf projects
include nesper/build_utils/tasks
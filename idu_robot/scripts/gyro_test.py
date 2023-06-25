import smbus
import time

# MPU6050 Registers and Addresses
DEVICE_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Configurable parameters
GYRO_SCALE = 131.0  # Gyro sensitivity, can be 131, 65.5, 32.8, or 16.4

# Initialize I2C bus
bus = smbus.SMBus(1)

# Wake up MPU-6050
bus.write_byte_data(DEVICE_ADDR, PWR_MGMT_1, 0)

# Calculate the offset
offset = [0, 0, 0]
n_samples = 1000

for i in range(n_samples):
    # Read the raw gyro values
    x = (bus.read_byte_data(DEVICE_ADDR, GYRO_XOUT_H) << 8) | bus.read_byte_data(DEVICE_ADDR, GYRO_XOUT_H + 1)
    y = (bus.read_byte_data(DEVICE_ADDR, GYRO_YOUT_H) << 8) | bus.read_byte_data(DEVICE_ADDR, GYRO_YOUT_H + 1)
    z = (bus.read_byte_data(DEVICE_ADDR, GYRO_ZOUT_H) << 8) | bus.read_byte_data(DEVICE_ADDR, GYRO_ZOUT_H + 1)

    # Convert the raw values to degrees per second
    x = (x / GYRO_SCALE)
    y = (y / GYRO_SCALE)
    z = (z / GYRO_SCALE)

    # Accumulate the values for the offset
    offset[0] += x
    offset[1] += y
    offset[2] += z

    # Wait a short period of time between readings
    time.sleep(0.001)

# Calculate the average offset
offset[0] /= n_samples
offset[1] /= n_samples
offset[2] /= n_samples

# Print the offset values
print("Gyro offset: ", offset)


// PlanetX Trackbit line sensor: four reflectance channels on I2C 0x1A,
// channel 0 on the robot's left.
namespace calibrate {
    // Where each channel sits, in cm left of the robot's centre line.
    export const BAR_LATERAL = [3.0, 0.6, -0.6, -3.0]

    // One bit per channel, set when that channel sees the line.
    export function lineBits(): number {
        pins.i2cWriteNumber(0x1a, 4, NumberFormat.Int8LE)
        return pins.i2cReadNumber(0x1a, NumberFormat.UInt8LE, false) & 0x0f
    }
}

// The four-channel line sensor, channel 0 on the robot's left.
// Nezha: the PlanetX Trackbit on I2C 0x1A. Cutebot Pro: the built-in
// sensor behind the board's 0x10 MCU. Both are read through
// diffDrive.lineSensorBits(), which holds the drive fiber's bus guard.
namespace calibrate {
    // Where each channel sits, in cm left of the robot's centre line.
    export let BAR_LATERAL = [3.0, 0.6, -0.6, -3.0]
    const CUTEBOT_LATERAL = [2.4, 0.8, -0.8, -2.4]
    let lateralSet = false

    function chooseLateral() {
        if (lateralSet) return
        lateralSet = true
        if (diffDrive.boardKind() == 2) BAR_LATERAL = CUTEBOT_LATERAL
    }

    // One bit per channel, set when that channel sees the line.
    export function lineBits(): number {
        chooseLateral()
        const bits = diffDrive.lineSensorBits()
        return bits < 0 ? 0 : bits & 0x0f
    }
}

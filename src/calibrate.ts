/**
 * Calibration programs for a Nezha robot driven by DiffDrive.
 */
//% color="#2E7D32" weight=80 icon="" block="Nezbot Calibrate"
namespace calibrate {
    let calibrationsSetUp: boolean
    let buttonsSetUp: boolean

    /**
     * Add the calibration programs as run commands: circle, square,
     * calwheels and calturn. The buttons are left alone.
     */
    //% block="setup calibrations"
    //% weight=100
    export function setupCalibrations(): void {
        if (calibrationsSetUp) return
        calibrationsSetUp = true

        diffDrive.addRunProgram("circle", driveCircle)
        diffDrive.runSignature("circle", "()")

        diffDrive.addRunProgram("square", driveSquare)
        diffDrive.runSignature("square", "()")

        diffDrive.addRunProgram("calwheels", runCalWheels)
        diffDrive.runSignature("calwheels", "(cm:number=90.5, wheel:number=0)")

        diffDrive.addRunProgram("calturn", runCalTurn)
        diffDrive.runSignature("calturn", "(edges:number=10)")

        diffDrive.onRun("_caldump", function (arg: number) { dump() })
        diffDrive.runSignature("_caldump", "()")
        diffDrive.onRun("_calcode", function (arg: number) { dumpCode() })
        diffDrive.runSignature("_calcode", "()")
    }

    /**
     * Put the calibration programs on the buttons: A steps through
     * them, B runs the one shown, and any button stops a running
     * program. Takes over the A, B and A+B buttons.
     */
    //% block="setup calibration buttons"
    //% weight=95
    export function setupButtons(): void {
        if (buttonsSetUp) return
        buttonsSetUp = true
        diffDrive.addMenuProgram("circle", SHAPE_CIRCLE, driveCircle)
        diffDrive.addMenuProgram("square", SHAPE_SQUARE, driveSquare)
        diffDrive.addMenuProgram("calwheels", SHAPE_OUT_AND_BACK, runCalWheels)
        diffDrive.addMenuProgram("calturn", SHAPE_SPIN, runCalTurn)
        diffDrive.emitLine("boot buttons: A=pick program  B=run it")
    }

    /**
     * Write the calibration the robot is driving on to the serial port,
     * as one JSON line.
     */
    //% block="dump calibration"
    //% weight=90
    export function dump(): void {
        const wheel = diffDrive.wheelCalibration()
        diffDrive.report("cal.values")
            .str("name", control.deviceName())
            .num("wheel", wheel, 4)
            .num("diameter", wheel * 360 / Math.PI, 2)
            .num("tw", diffDrive.trackWidth(), 2)
            .num("slip", diffDrive.rotationalSlip(), 4)
            .num("scale_l", diffDrive.wheelMultiplier(MotorSide.Left), 3)
            .num("scale_r", diffDrive.wheelMultiplier(MotorSide.Right), 3)
            .num("port_l", diffDrive.configValue(ConfigField.MotorPortLeft), 0)
            .num("port_r", diffDrive.configValue(ConfigField.MotorPortRight), 0)
            .send()
        diffDrive.flushReports()
    }

    /**
     * Write the calibration the robot is driving on to the serial port,
     * as code to paste into a program.
     */
    //% block="dump calibration code"
    //% weight=80
    export function dumpCode(): void {
        const portLeft = diffDrive.configValue(ConfigField.MotorPortLeft)
        const portRight = diffDrive.configValue(ConfigField.MotorPortRight)
        diffDrive.emitLine("// calibration for " + control.deviceName())
        diffDrive.emitLine("diffDrive.setMotorPorts(MotorPort.M" + portLeft
            + ", MotorPort.M" + portRight + ")")
        diffDrive.emitLine("diffDrive.setWheelMultiplier(MotorSide.Left, "
            + diffDrive.roundTo(diffDrive.wheelMultiplier(MotorSide.Left), 3) + ")")
        diffDrive.emitLine("diffDrive.setWheelMultiplier(MotorSide.Right, "
            + diffDrive.roundTo(diffDrive.wheelMultiplier(MotorSide.Right), 3) + ")")
        diffDrive.emitLine("diffDrive.setWheelCalibration("
            + diffDrive.roundTo(diffDrive.wheelCalibration(), 4) + ")")
        diffDrive.emitLine("diffDrive.setTrackWidth("
            + diffDrive.roundTo(diffDrive.trackWidth(), 2) + ")")
        diffDrive.emitLine("diffDrive.setRotationalSlip("
            + diffDrive.roundTo(diffDrive.rotationalSlip(), 4) + ")")
    }
}

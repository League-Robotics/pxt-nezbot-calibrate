/**
 * Calibration programs for a Nezha robot driven by DiffDrive.
 */
//% color="#2E7D32" weight=80 icon="" block="Nezbot Calibrate"
namespace calibrate {
    let registered: boolean

    /**
     * Bring the robot up and add the calibration programs to the button
     * menu and the run commands: circle, square, calwheels and calturn.
     * A steps through the programs, B runs the one shown, and any
     * button stops a running program.
     */
    //% block="register calibration programs"
    //% weight=100
    export function registerPrograms(): void {
        if (registered) return
        registered = true
        diffDrive.setupRobot()

        diffDrive.addProgram("circle", SHAPE_CIRCLE, driveCircle)
        diffDrive.runSignature("circle", "()")

        diffDrive.addProgram("square", SHAPE_SQUARE, driveSquare)
        diffDrive.runSignature("square", "()")

        diffDrive.addProgram("calwheels", SHAPE_OUT_AND_BACK, runCalWheels)
        diffDrive.runSignature("calwheels", "(cm:number=90.5, wheel:number=0)")

        diffDrive.addProgram("calturn", SHAPE_SPIN, runCalTurn)
        diffDrive.runSignature("calturn", "(edges:number=10)")

        diffDrive.onRun("_caldump", function (arg: number) { dump() })
        diffDrive.runSignature("_caldump", "()")
        diffDrive.onRun("_calcode", function (arg: number) { dumpCode() })
        diffDrive.runSignature("_calcode", "()")

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
        diffDrive.emitLine("diffDrive.setConfigValue(ConfigField.RotationalSlip, "
            + diffDrive.roundTo(diffDrive.rotationalSlip(), 4) + ")")
    }
}

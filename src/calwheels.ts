// Wheel calibration: drive between two lines a tape-measured distance apart,
// straddling the edge of the centre stripe, and compare with the odometry.
namespace calibrate {
    const WHEELS_BASELINE = 0.7878  // mm/deg, the engine's default
    const WHEELS_TRUE_CM = 90.2     // tape-measured, start line to finish line
    const WHEELS_STRIPE_W = 1.68    // cm, centre stripe width
    const WHEELS_SPEED = 8          // cm/s
    const WHEELS_KP = 1.1           // cm/s of wheel differential per cm of error
    const WHEELS_MAX_STEER = 2.5    // cm/s
    const WHEELS_DEADBAND = 0.3     // cm
    const WHEELS_BLIND_ERR = 1.2    // cm of error assumed while no channel sees tape
    const WHEELS_ACQ_HALF = 14      // ticks per half of the search sweep
    const WHEELS_ACQ_STEER = 1.4    // cm/s differential while searching
    const WHEELS_ACQ_MAX = 150      // ticks searching before giving up
    const WHEELS_BLIND_MAX = 40     // blind ticks in a row before stopping
    const WHEELS_MID_MAX = 40       // all-dark ticks in a row before stopping
    const WHEELS_PHASE2_CLEAR = 4   // cm past the start line
    const WHEELS_TOL_FRAC = 0.1     // allowed error against a declared wheel
    const WHEELS_DIA_MIN = 40       // mm
    const WHEELS_DIA_MAX = 120      // mm
    const WHEELS_HUNT_CM = 25       // cm to find the start line
    const WHEELS_EXTRA_CM = 25      // cm of slack past the expected finish
    const WHEELS_MAX_SECS = 60
    const WHEELS_DEAD_TICKS = 25    // ticks with no encoder change before stopping
    const WHEELS_TRK_EVERY = 25     // ticks between tracking lines
    const WHEELS_ALL = 15           // all four channel bits
    const WHEELS_OUTER = 9          // channels 0 and 3

    // The reverse gains are negative: steering acts the other way going backwards.
    const WHEELS_BACK_SPEED = 11    // cm/s
    const WHEELS_BACK_CLEAR = 3     // cm past the start line before stopping
    const WHEELS_BACK_MAX = 110     // cm
    const WHEELS_BACK_KP = -0.7     // cm/s per cm of error
    const WHEELS_BACK_KH = -0.22    // cm/s per degree of heading error
    const WHEELS_BACK_KI = 0
    const WHEELS_BACK_IMAX = 60
    const WHEELS_BACK_MAX_STEER = 2.5
    const WHEELS_BACK_BLIND_MAX = 25  // blind ticks in a row before stopping

    export function barText(bits: number): string {
        let text = ""
        for (let i = 0; i < 4; i++) text = text + ((bits & (1 << i)) != 0 ? "#" : ".")
        return text
    }

    function stripeTarget(): number {
        return BAR_LATERAL[2] + WHEELS_STRIPE_W / 2
    }

    function stripeEdge(bits: number): number {
        let sum = 0
        let n = 0
        for (let i = 0; i < 4; i++) {
            if ((bits & (1 << i)) != 0) { sum += BAR_LATERAL[i]; n++ }
        }
        if (n == 0) return 999
        return sum / n + WHEELS_STRIPE_W / 2
    }

    function calibOfDiameter(diaMm: number): number {
        return Math.PI * diaMm / 360
    }

    /**
     * Measure the wheels: start on clear white before the first line
     * and the robot follows the stripe to the second line, then keeps
     * the result and backs up to where it started.
     * @param trueCm tape-measured distance between the two lines in cm, eg: 90.2
     * @param wheelMm wheel diameter in mm if you know it, 0 if not, eg: 0
     */
    //% block="calibrate wheels over %trueCm cm || with wheel diameter %wheelMm mm"
    //% expandableArgumentMode="toggle"
    //% weight=70
    export function calibrateWheels(trueCm: number, wheelMm: number = 0) {
        const known = wheelMm > 0
        const runCalib = known ? calibOfDiameter(wheelMm) : WHEELS_BASELINE

        const spanLo = known ? 1 - WHEELS_TOL_FRAC : runCalib / calibOfDiameter(WHEELS_DIA_MAX)
        const spanHi = known ? 1 + WHEELS_TOL_FRAC : runCalib / calibOfDiameter(WHEELS_DIA_MIN)

        diffDrive.setWheelCalibration(runCalib)
        diffDrive.emitLine("CALWHEELS:begin true=" + trueCm + "cm scale=" + diffDrive.roundTo(runCalib, 4)
            + "mm/deg wheel=" + (known ? wheelMm + "mm" : "unknown")
            + " span=" + diffDrive.roundTo(trueCm * spanLo, 1) + ".." + diffDrive.roundTo(trueCm * spanHi, 1) + "cm"
            + " speed=" + WHEELS_SPEED + " kp=" + WHEELS_KP
            + " dead=" + WHEELS_DEADBAND + " aim=" + diffDrive.roundTo(stripeTarget(), 2) + "cm")

        const atStart = lineBits()
        if (atStart != 0) {
            diffDrive.report("calwheels.fail").str("why", "not on clear white")
                .str("bar", barText(atStart)).send()
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }

        diffDrive.resetPose()
        let phase = 1
        let xA = 0
        let xB = 0
        let lastX = 0
        let trackTick = 0
        let bailed = ""
        let nTicks = 0
        let sumAbs = 0
        let sumSq = 0
        let maxAbs = 0
        let crossings = 0
        let lastSign = 0
        let sumDiff = 0
        let blindTicks = 0
        let blindRun = 0
        let acquired = false
        let acqTicks = 0
        let deadRun = 0
        let midTicks = 0
        let lastEnc = diffDrive.probe(10) + diffDrive.probe(11)
        const startedAt = control.millis()

        diffDrive.setWheelSpeeds(WHEELS_SPEED, WHEELS_SPEED)
        while (diffDrive.driveTick()) {
            if (diffDrive.cancelled()) { bailed = "stopped by a button press"; break }
            const x = diffDrive.poseX()
            const bits = lineBits()

            if (phase == 1) {
                if (bits == WHEELS_ALL) {
                    xA = (lastX + x) / 2
                    phase = 2
                    diffDrive.emitLine("CALWHEELS:start line at=" + diffDrive.roundTo(xA, 2) + "cm")
                } else if (x >= WHEELS_HUNT_CM) {
                    bailed = "no start line within " + WHEELS_HUNT_CM + "cm (bar=" + barText(bits) + ")"
                    break
                }
            } else if (phase == 2) {
                if ((bits & WHEELS_OUTER) == 0 || x - xA >= WHEELS_PHASE2_CLEAR) {
                    phase = 3
                    diffDrive.emitLine("CALWHEELS:straddling from=" + diffDrive.roundTo(x, 2)
                        + "cm bar=" + barText(bits))
                }
            } else {
                if (bits == WHEELS_ALL && x - xA >= trueCm * spanLo) {
                    xB = (lastX + x) / 2
                    phase = 4
                    break
                }

                if (bits == WHEELS_ALL) {
                    midTicks++
                    if (midTicks == 1) {
                        diffDrive.emitLine("CALWHEELS:mid-field line at "
                            + diffDrive.roundTo(x - xA, 1) + "cm -- holding course, not the finish")
                    }
                    if (midTicks >= WHEELS_MID_MAX) {
                        bailed = "all four channels dark for " + midTicks + " ticks at "
                            + diffDrive.roundTo(x - xA, 1) + "cm -- that is not a line, it is"
                            + " the robot off the paper"
                        break
                    }
                    diffDrive.setWheelSpeeds(WHEELS_SPEED, WHEELS_SPEED)
                    lastX = x
                    continue
                }
                midTicks = 0

                const edge = stripeEdge(bits)
                let err = 0
                if (edge > 900) {
                    blindTicks++
                    if (!acquired) {
                        acqTicks++
                        if (acqTicks >= WHEELS_ACQ_MAX) {
                            bailed = "never found the stripe in " + acqTicks
                                + " ticks of searching from " + diffDrive.roundTo(x - xA, 1) + "cm"
                            break
                        }
                        const phaseHalf = Math.idiv(acqTicks, WHEELS_ACQ_HALF)
                        const dir = phaseHalf % 2 == 0 ? 1 : -1
                        const grow = 1 + phaseHalf / 4
                        let sSteer = dir * WHEELS_ACQ_STEER * grow
                        if (sSteer > WHEELS_MAX_STEER) sSteer = WHEELS_MAX_STEER
                        if (sSteer < -WHEELS_MAX_STEER) sSteer = -WHEELS_MAX_STEER
                        diffDrive.setWheelSpeeds(WHEELS_SPEED - sSteer, WHEELS_SPEED + sSteer)
                        lastX = x
                        continue
                    }
                    blindRun++
                    if (blindRun >= WHEELS_BLIND_MAX) {
                        bailed = "lost the stripe for " + blindRun + " ticks at "
                            + diffDrive.roundTo(x - xA, 1) + "cm -- stopping rather than arcing away"
                        break
                    }
                    err = lastSign >= 0 ? WHEELS_BLIND_ERR : -WHEELS_BLIND_ERR
                } else {
                    acquired = true
                    blindRun = 0
                    err = edge - stripeTarget()
                }

                nTicks++
                const mag = Math.abs(err)
                sumAbs += mag
                sumSq += err * err
                if (mag > maxAbs) maxAbs = mag
                const sign = err > WHEELS_DEADBAND ? 1 : (err < -WHEELS_DEADBAND ? -1 : 0)
                if (sign != 0) {
                    if (lastSign != 0 && sign != lastSign) crossings++
                    lastSign = sign
                }

                let steer = 0
                if (mag > WHEELS_DEADBAND) {
                    steer = WHEELS_KP * err
                    if (steer > WHEELS_MAX_STEER) steer = WHEELS_MAX_STEER
                    if (steer < -WHEELS_MAX_STEER) steer = -WHEELS_MAX_STEER
                }
                sumDiff += -2 * steer
                diffDrive.setWheelSpeeds(WHEELS_SPEED - steer, WHEELS_SPEED + steer)

                trackTick++
                if (trackTick % WHEELS_TRK_EVERY == 0) {
                    diffDrive.emitLine("CALWHEELS:trk x=" + diffDrive.roundTo(x - xA, 1)
                        + "cm bar=" + barText(bits) + " err=" + diffDrive.roundTo(err, 2)
                        + " steer=" + diffDrive.roundTo(steer, 2))
                }
            }

            const encNow = diffDrive.probe(10) + diffDrive.probe(11)
            if (encNow == lastEnc) {
                deadRun++
                if (deadRun >= WHEELS_DEAD_TICKS) {
                    bailed = "encoders are DEAD -- posl+posr has not changed in "
                        + deadRun + " commanded ticks, so every distance limit in this"
                        + " run is inert. Check i2cf against cyc in TLM FULL:"
                        + " near-equal means the I2C bus is failing."
                    break
                }
            } else {
                deadRun = 0
            }
            lastEnc = encNow

            lastX = x
            if (phase > 1 && x - xA >= trueCm * spanHi + WHEELS_EXTRA_CM) {
                bailed = "ran " + diffDrive.roundTo(x - xA, 1) + "cm past the start line"
                    + " without finding the finish"
                break
            }
            if (control.millis() - startedAt > WHEELS_MAX_SECS * 1000) {
                bailed = "timed out after " + WHEELS_MAX_SECS + "s at " + diffDrive.roundTo(x, 1) + "cm"
                break
            }
        }
        diffDrive.stop()

        if (phase != 4) {
            diffDrive.report("calwheels.fail")
                .str("why", bailed.length > 0 ? bailed : "drive ended early")
                .num("phase", phase, 0).send()
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }

        const measured = xB - xA

        const corrected = measured > 0 ? runCalib * trueCm / measured : 0
        const diameter = corrected * 360 / Math.PI

        if (measured < trueCm * spanLo || measured > trueCm * spanHi) {
            const bad = diffDrive.report("calwheels.fail")
            bad.str("why", known
                ? "measured distance is nowhere near the declared wheel"
                : "no wheel that fits this chassis could have driven that")
            bad.num("measured", measured, 2)
            bad.num("true", trueCm, 2)
            bad.num("implied", diameter, 2)
            bad.num("lo", trueCm * spanLo, 2)
            bad.num("hi", trueCm * spanHi, 2)
            bad.str("wheel", known ? "" + wheelMm : "unknown")
            bad.send()
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }
        const meanAbs = nTicks > 0 ? sumAbs / nTicks : 0
        const rms = nTicks > 0 ? Math.sqrt(sumSq / nTicks) : 0
        const bias = nTicks > 0 ? sumDiff / nTicks : 0
        const perM = measured > 0 ? crossings * 100 / measured : 0

        diffDrive.saveWheelCalibration(corrected)

        const r = diffDrive.report("calwheels.result")
        r.num("calib", corrected, 4)
        r.num("diameter", diameter, 2)
        r.num("measured", measured, 2)
        r.num("true", trueCm, 2)
        r.num("error", measured - trueCm, 2)
        r.num("was", runCalib, 4)
        r.str("wheel", known ? "" + wheelMm : "unknown")
        r.num("stored", 1, 0)
        r.send()

        const q = diffDrive.report("calwheels.quality")
        q.num("rms", rms, 2)
        q.num("mean", meanAbs, 2)
        q.num("max", maxAbs, 2)
        q.num("xpm", perM, 1)
        q.num("blind", blindTicks, 0)
        q.num("ticks", nTicks, 0)
        q.num("acq", acqTicks, 0)
        q.num("bias", bias, 3)
        q.num("heading", diffDrive.heading(), 2)
        q.send()

        const w = diffDrive.report("calwheels.span")
        w.num("start", xA, 2)
        w.num("finish", xB, 2)
        w.send()
        diffDrive.flushReports()
        basic.showIcon(IconNames.Yes)
        driveHome(trueCm)
    }

    function driveHome(minCm: number) {
        diffDrive.emitLine("CALWHEELS:home reverse PID kp=" + WHEELS_BACK_KP + " kh=" + WHEELS_BACK_KH
            + " ki=" + WHEELS_BACK_KI + " speed=" + WHEELS_BACK_SPEED + "cm/s")
        diffDrive.resetPose()
        let stage = lineBits() == WHEELS_ALL ? 0 : 1
        let crossedAt = 0
        let integral = 0
        let blindRun = 0
        let blindTicks = 0
        let nTicks = 0
        let sumAbs = 0
        let sumSq = 0
        let maxAbs = 0
        let crossings = 0
        let lastSign = 0
        let trk = 0
        const startedAt = control.millis()

        diffDrive.setWheelSpeeds(-WHEELS_BACK_SPEED, -WHEELS_BACK_SPEED)
        while (diffDrive.driveTick()) {
            if (diffDrive.cancelled()) {
                diffDrive.stop()
                diffDrive.emitLine("CALWHEELS:home stopped by a button press")
                return
            }
            const x = Math.abs(diffDrive.poseX())
            const bits = lineBits()

            if (stage == 0) {
                if ((bits & WHEELS_OUTER) == 0) stage = 1
            } else if (stage == 1) {
                if (bits == WHEELS_ALL && x >= minCm * (1 - WHEELS_TOL_FRAC)) {
                    stage = 2
                    crossedAt = x
                }
            } else {
                if (bits == 0 && x - crossedAt >= WHEELS_BACK_CLEAR) break
            }

            if (stage == 1 && bits == WHEELS_ALL) {
                diffDrive.setWheelSpeeds(-WHEELS_BACK_SPEED, -WHEELS_BACK_SPEED)
            } else if (stage == 1) {
                const edge = stripeEdge(bits)
                let err = 0
                if (edge > 900) {
                    blindTicks++
                    blindRun++
                    if (blindRun >= WHEELS_BACK_BLIND_MAX) {
                        diffDrive.stop()
                        diffDrive.emitLine("CALWHEELS:home fail lost the stripe for " + blindRun
                            + " ticks at " + diffDrive.roundTo(x, 1) + "cm -- stopping, not guessing")
                        basic.showIcon(IconNames.No)
                        return
                    }
                    err = lastSign >= 0 ? WHEELS_BLIND_ERR : -WHEELS_BLIND_ERR
                } else {
                    blindRun = 0
                    err = edge - stripeTarget()
                }

                nTicks++
                const mag = Math.abs(err)
                sumAbs += mag
                sumSq += err * err
                if (mag > maxAbs) maxAbs = mag
                const sign = err > WHEELS_DEADBAND ? 1 : (err < -WHEELS_DEADBAND ? -1 : 0)
                if (sign != 0) {
                    if (lastSign != 0 && sign != lastSign) crossings++
                    lastSign = sign
                }

                integral += err
                if (integral > WHEELS_BACK_IMAX) integral = WHEELS_BACK_IMAX
                if (integral < -WHEELS_BACK_IMAX) integral = -WHEELS_BACK_IMAX

                let steer = WHEELS_BACK_KP * err + WHEELS_BACK_KH * diffDrive.heading()
                    + WHEELS_BACK_KI * integral
                if (steer > WHEELS_BACK_MAX_STEER) steer = WHEELS_BACK_MAX_STEER
                if (steer < -WHEELS_BACK_MAX_STEER) steer = -WHEELS_BACK_MAX_STEER
                diffDrive.setWheelSpeeds(-WHEELS_BACK_SPEED - steer, -WHEELS_BACK_SPEED + steer)

                trk++
                if (trk % WHEELS_TRK_EVERY == 0) {
                    diffDrive.emitLine("CALWHEELS:hometrk x=" + diffDrive.roundTo(x, 1) + "cm bar="
                        + barText(bits) + " err=" + diffDrive.roundTo(err, 2)
                        + " h=" + diffDrive.roundTo(diffDrive.heading(), 1)
                        + " steer=" + diffDrive.roundTo(steer, 2))
                }
            }

            if (x >= WHEELS_BACK_MAX) {
                diffDrive.stop()
                diffDrive.emitLine("CALWHEELS:home fail no line within " + WHEELS_BACK_MAX
                    + "cm (bar=" + barText(bits) + ")")
                basic.showIcon(IconNames.No)
                return
            }
            if (control.millis() - startedAt > WHEELS_MAX_SECS * 1000) {
                diffDrive.stop()
                diffDrive.emitLine("CALWHEELS:home fail timed out at " + diffDrive.roundTo(x, 1) + "cm")
                basic.showIcon(IconNames.No)
                return
            }
        }
        diffDrive.stop()

        const meanAbs = nTicks > 0 ? sumAbs / nTicks : 0
        const rms = nTicks > 0 ? Math.sqrt(sumSq / nTicks) : 0
        diffDrive.emitLine("CALWHEELS:home back " + diffDrive.roundTo(Math.abs(diffDrive.poseX()), 2)
            + "cm bar=" + barText(lineBits())
            + " heading=" + diffDrive.roundTo(diffDrive.heading(), 2) + "deg")
        diffDrive.emitLine("CALWHEELS:homeosc rms=" + diffDrive.roundTo(rms, 2) + "cm mean="
            + diffDrive.roundTo(meanAbs, 2) + "cm max=" + diffDrive.roundTo(maxAbs, 2) + "cm crossings="
            + crossings + " blind=" + blindTicks + "/" + nTicks + "ticks")
        basic.showIcon(IconNames.Yes)
    }

    // RUN calwheels [cm [wheel]]; from the button menu it runs on the defaults.
    export function runCalWheels() {
        calibrateWheels(diffDrive.runArg(0) || WHEELS_TRUE_CM, diffDrive.runArg(1))
    }
}

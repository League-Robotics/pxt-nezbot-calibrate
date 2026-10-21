// Turn calibration: spin on the iron cross and time each sensor channel's
// crossings of the 45-degree sector edges. The odometry is run on a known
// anchor geometry, so the measured sector size gives the true effective track.
namespace calibrate {
    const TURN_TRACK = 11.42       // cm, anchor track width
    const TURN_SLIP = 0.952        // anchor slip
    const TURN_SECTOR = 45         // deg between sector edges
    const TURN_SPIN = 70           // deg/s; slower and the reversing wheel stalls
    const CUTEBOT_TURN_SPIN = 115  // deg/s; the Cutebot Pro stick-slips to a stop at 70
    const TURN_EDGES = 10          // edges per channel; the first gap is discarded
    const TURN_MAX_SWEEP = 2000    // deg
    const TURN_MAX_SECS = 120
    const TURN_STUCK_DEG = 2       // deg of heading that counts as still turning
    const TURN_STUCK_MS = 3000     // ms without that much turn before giving up
    const TURN_SLIP_MIN = 0.8      // a slip outside these means the wheels spun
    const TURN_SLIP_MAX = 1.25     // or the robot was held, not a measurement
    const TURN_PAIR_TOL = 0.1      // a left/right pair further apart is not averaged
    const TURN_SETTLE = 300        // ms of stillness before the spin

    const LAST_SLIP_KEY = "#ct.last"   // slip measured by the previous run
    const LAST_DIR_KEY = "#ct.ldir"    // its spin direction, +1 left, -1 right
    let edges0: number[] = []
    let edges1: number[] = []
    let edges2: number[] = []
    let edges3: number[] = []

    function edgesFor(ch: number): number[] {
        if (ch == 0) return edges0
        if (ch == 1) return edges1
        if (ch == 2) return edges2
        return edges3
    }

    function meanEdgeGap(list: number[]): number {
        if (list.length < 3) return -1
        let sum = 0
        let n = 0
        for (let i = 2; i < list.length; i++) {
            sum += list[i] - list[i - 1]
            n++
        }
        return n > 0 ? sum / n : -1
    }

    function edgeGapSpread(list: number[]): number {
        const mean = meanEdgeGap(list)
        if (mean < 0) return -1
        let sumSq = 0
        let n = 0
        for (let i = 2; i < list.length; i++) {
            const d = (list[i] - list[i - 1]) - mean
            sumSq += d * d
            n++
        }
        return n > 1 ? Math.sqrt(sumSq / (n - 1)) : 0
    }

    function reportTurnGeometry(stored: number) {
        diffDrive.report("calturn.restored")
            .num("tw", diffDrive.trackWidth(), 2)
            .num("slip", diffDrive.rotationalSlip(), 4)
            .num("stored", stored, 0)
            .send()
    }

    /**
     * Measure the track: centre the robot on the iron cross and it
     * spins in place, then keeps the result. Each run spins the
     * opposite way from the last, so two runs measure both directions.
     * @param edgesWanted sector edges to time on each sensor channel, eg: 10
     */
    //% block="calibrate turn || over %edgesWanted edges"
    //% expandableArgumentMode="toggle"
    //% weight=60
    export function calibrateTurn(edgesWanted: number = 10) {
        // A whole number of revolutions, so a centring error cancels.
        const revs = Math.max(1, Math.round((edgesWanted - 2) / 8))
        const edges = 2 + 8 * revs
        if (edges != edgesWanted) {
            diffDrive.emitLine("CALTURN:edges " + edgesWanted + " -> " + edges
                + " (the window must be a whole number of revolutions: 8n+2)")
        }
        const spin = diffDrive.boardKind() == 2 ? CUTEBOT_TURN_SPIN : TURN_SPIN
        const trackWidth = diffDrive.trackWidth()
        const slip = diffDrive.rotationalSlip()
        diffDrive.setTrackWidth(TURN_TRACK)
        diffDrive.setConfigValue(ConfigField.RotationalSlip, TURN_SLIP)
        const bAnchor = TURN_TRACK / TURN_SLIP
        const lastDir = settings.exists(LAST_DIR_KEY) ? settings.readNumber(LAST_DIR_KEY) : -1
        const dir = lastDir > 0 ? -1 : 1
        diffDrive.emitLine("CALTURN:begin edges=" + edges + " sector=" + TURN_SECTOR
            + "deg anchor b=" + diffDrive.roundTo(bAnchor, 3) + "cm (track " + TURN_TRACK
            + " slip " + TURN_SLIP + ") spin=" + spin + "deg/s turn="
            + (dir > 0 ? "left" : "right"))

        edges0 = []; edges1 = []; edges2 = []; edges3 = []

        const wheel = spin * Math.PI / 180 * bAnchor / 2

        basic.pause(TURN_SETTLE)
        diffDrive.resetPose()
        let last = lineBits()
        let done = 0
        const startedAt = control.millis()
        diffDrive.emitLine("CALTURN:spin start bar=" + barText(last)
            + " wheel=" + diffDrive.roundTo(wheel, 2) + "cm/s")

        diffDrive.setWheelSpeeds(-dir * wheel, dir * wheel)
        let cancelled = false
        let stuck = false
        let progressH = 0
        let progressAt = control.millis()
        while (diffDrive.driveTick()) {
            if (diffDrive.cancelled()) { cancelled = true; break }
            const h = diffDrive.heading() * dir
            const bits = lineBits()
            if (bits != last) {
                for (let i = 0; i < 4; i++) {
                    const bit = 1 << i
                    if ((bits & bit) != (last & bit)) {
                        const list = edgesFor(i)
                        if (list.length < edges) {
                            list.push(h)
                            diffDrive.emitLine("CALTURN:ch" + i + " n=" + list.length
                                + " h=" + diffDrive.roundTo(h, 2) + "deg bar=" + barText(bits))
                        }
                    }
                }
                last = bits
                done = 0
                for (let j = 0; j < 4; j++) {
                    if (edgesFor(j).length >= edges) done++
                }
                if (done == 4) break
            }
            if (Math.abs(h - progressH) >= TURN_STUCK_DEG) {
                progressH = h
                progressAt = control.millis()
            } else if (control.millis() - progressAt > TURN_STUCK_MS) {
                stuck = true
                break
            }
            if (Math.abs(h) >= TURN_MAX_SWEEP) break
            if (control.millis() - startedAt > TURN_MAX_SECS * 1000) break
            diffDrive.setWheelSpeeds(-dir * wheel, dir * wheel)
        }
        diffDrive.stop()

        diffDrive.setTrackWidth(trackWidth)
        diffDrive.setConfigValue(ConfigField.RotationalSlip, slip)

        if (cancelled) {
            diffDrive.reportStopped("calturn")
            return
        }
        if (stuck || done < 4) {
            diffDrive.report("calturn.fail")
                .str("why", stuck ? "stopped turning; nothing stored" : "spin ended before every channel had its edges; nothing stored")
                .num("h", diffDrive.heading() * dir, 1)
                .send()
            reportTurnGeometry(0)
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }

        let grand = 0
        let grandN = 0
        let worstSd = 0
        let chLo = 1e9
        let chHi = -1e9
        let chUsable = 0
        for (let c = 0; c < 4; c++) {
            const list = edgesFor(c)
            const mean = meanEdgeGap(list)
            if (mean < 0) {
                diffDrive.report("calturn.ch").str("state", "unusable")
                    .num("i", c, 0).num("n", list.length, 0).send()
                continue
            }
            const sd = edgeGapSpread(list)
            if (sd > worstSd) worstSd = sd
            if (mean < chLo) chLo = mean
            if (mean > chHi) chHi = mean
            chUsable++
            diffDrive.report("calturn.ch")
                .num("i", c, 0)
                .num("n", list.length, 0)
                .num("gap", mean, 3)
                .num("sd", sd, 3)
                .num("slope", mean / TURN_SECTOR, 4)
                .send()
            grand += mean * (list.length - 2)
            grandN += list.length - 2
        }

        if (grandN < 4) {
            diffDrive.report("calturn.fail")
                .num("gaps", grandN, 0)
                .str("why", "too few usable gaps; centre the robot on the cross")
                .send()
            reportTurnGeometry(0)
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }

        const meanGap = grand / grandN
        const slope = meanGap / TURN_SECTOR
        const bTrue = bAnchor * slope

        diffDrive.report("calturn.result")
            .num("b", bTrue, 3)
            .num("tw", trackWidth, 2)
            .num("slip", trackWidth / bTrue, 4)
            .num("anchor_tw", TURN_TRACK, 2)
            .num("slip_at_anchor", TURN_TRACK / bTrue, 4)
            .num("slope", slope, 4)
            .num("gaps", grandN, 0)
            .num("anchor_b", bAnchor, 3)
            .send()
        diffDrive.report("calturn.quality")
            .num("sd", worstSd, 3)
            .num("spread", chUsable > 0 ? chHi - chLo : 0, 3)
            .num("ch", chUsable, 0)
            .num("gap", meanGap, 3)
            .num("sector", TURN_SECTOR, 0)
            .num("spin", spin, 0)
            .num("wheel", wheel, 2)
            .send()
        // Left and right spins disagree by a few percent, so keep the mean
        // of this run and the previous one when that one spun the other way.
        const runSlip = trackWidth / bTrue
        if (runSlip < TURN_SLIP_MIN || runSlip > TURN_SLIP_MAX) {
            diffDrive.report("calturn.fail")
                .str("why", "slip " + diffDrive.roundTo(runSlip, 3) + " is not plausible; the wheels slipped or the robot was held -- nothing stored")
                .send()
            reportTurnGeometry(0)
            diffDrive.flushReports()
            basic.showIcon(IconNames.No)
            return
        }
        let slipToStore = runSlip
        if (settings.exists(LAST_SLIP_KEY) && lastDir == -dir) {
            const last = settings.readNumber(LAST_SLIP_KEY)
            if (Math.abs(runSlip - last) <= TURN_PAIR_TOL * last) slipToStore = (runSlip + last) / 2
        }
        settings.writeNumber(LAST_SLIP_KEY, runSlip)
        settings.writeNumber(LAST_DIR_KEY, dir)
        diffDrive.report("calturn.stored")
            .num("run", runSlip, 4)
            .num("slip", slipToStore, 4)
            .str("pair", slipToStore == runSlip ? "no" : "yes")
            .send()
        diffDrive.saveTurnCalibration(trackWidth, slipToStore)
        reportTurnGeometry(1)
        diffDrive.flushReports()
        basic.showIcon(IconNames.Yes)
    }

    // RUN calturn [edges]; from the button menu it runs on the default.
    export function runCalTurn() {
        calibrateTurn(diffDrive.runArg(0) || TURN_EDGES)
    }
}

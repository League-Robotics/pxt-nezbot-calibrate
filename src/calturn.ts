// Turn calibration: spin on the iron cross and time each sensor channel's
// crossings of the 45-degree sector edges. The odometry is run on a known
// anchor geometry, so the measured sector size gives the true effective track.
namespace calibrate {
    const TURN_TRACK = 11.42       // cm, anchor track width
    const TURN_SLIP = 0.952        // anchor slip
    const TURN_SECTOR = 45         // deg between sector edges
    const TURN_SPIN = 70           // deg/s; slower and the reversing wheel stalls
    const TURN_EDGES = 10          // edges per channel; the first gap is discarded
    const TURN_MAX_SWEEP = 2000    // deg
    const TURN_MAX_SECS = 120
    const TURN_SETTLE = 300        // ms of stillness before the spin

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

    function calibrateTurn(edgesWanted: number) {
        // A whole number of revolutions, so a centring error cancels.
        const revs = Math.max(1, Math.round((edgesWanted - 2) / 8))
        const edges = 2 + 8 * revs
        if (edges != edgesWanted) {
            diffDrive.emitLine("CALTURN:edges " + edgesWanted + " -> " + edges
                + " (the window must be a whole number of revolutions: 8n+2)")
        }
        const trackWidth = diffDrive.trackWidth()
        const slip = diffDrive.rotationalSlip()
        diffDrive.setTrackWidth(TURN_TRACK)
        diffDrive.setConfigValue(ConfigField.RotationalSlip, TURN_SLIP)
        const bAnchor = TURN_TRACK / TURN_SLIP
        diffDrive.emitLine("CALTURN:begin edges=" + edges + " sector=" + TURN_SECTOR
            + "deg anchor b=" + diffDrive.roundTo(bAnchor, 3) + "cm (track " + TURN_TRACK
            + " slip " + TURN_SLIP + ") spin=" + TURN_SPIN + "deg/s")

        edges0 = []; edges1 = []; edges2 = []; edges3 = []

        const wheel = TURN_SPIN * Math.PI / 180 * bAnchor / 2

        basic.pause(TURN_SETTLE)
        diffDrive.resetPose()
        let last = lineBits()
        let done = 0
        const startedAt = control.millis()
        diffDrive.emitLine("CALTURN:spin start bar=" + barText(last)
            + " wheel=" + diffDrive.roundTo(wheel, 2) + "cm/s")

        diffDrive.setWheelSpeeds(-wheel, wheel)
        let cancelled = false
        while (diffDrive.driveTick()) {
            if (diffDrive.cancelled()) { cancelled = true; break }
            const h = diffDrive.heading()
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
            if (Math.abs(h) >= TURN_MAX_SWEEP) break
            if (control.millis() - startedAt > TURN_MAX_SECS * 1000) break
            diffDrive.setWheelSpeeds(-wheel, wheel)
        }
        diffDrive.stop()

        diffDrive.setTrackWidth(trackWidth)
        diffDrive.setConfigValue(ConfigField.RotationalSlip, slip)

        if (cancelled) {
            diffDrive.reportStopped("calturn")
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
            .num("spin", TURN_SPIN, 0)
            .num("wheel", wheel, 2)
            .send()
        diffDrive.saveTurnCalibration(trackWidth, trackWidth / bTrue)
        reportTurnGeometry(1)
        diffDrive.flushReports()
        basic.showIcon(IconNames.Yes)
    }

    // RUN calturn [edges]; from the button menu it runs on the default.
    export function runCalTurn() {
        calibrateTurn(diffDrive.runArg(0) || TURN_EDGES)
    }
}

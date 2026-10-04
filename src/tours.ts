// Tours: a 50 cm square and a 30 cm radius circle.
namespace calibrate {
    export function driveSquare() {
        for (let i = 0; i < 4; i++) {
            basic.showArrow(ArrowNames.North, 0)
            if (!diffDrive.moveLeg(50, 0)) { diffDrive.reportStopped("square"); return }
            basic.showArrow(ArrowNames.West, 0)
            if (!diffDrive.moveLeg(0, 90)) { diffDrive.reportStopped("square"); return }
        }
        basic.clearScreen()
    }

    const CIRCLE_RADIUS = 30  // cm
    const CIRCLE_SEGMENT = CIRCLE_RADIUS * 45 * Math.PI / 180

    export function driveCircle() {
        for (let quarter = 1; quarter <= 4; quarter++) {
            basic.showNumber(quarter, 0)
            if (!diffDrive.moveLeg(CIRCLE_SEGMENT, 45)) { diffDrive.reportStopped("circle"); return }
            if (!diffDrive.moveLeg(CIRCLE_SEGMENT, 45)) { diffDrive.reportStopped("circle"); return }
        }
        basic.clearScreen()
    }
}

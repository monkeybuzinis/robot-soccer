/**
 * RobotPU soccer: pure pursuit waypoint follower.
 *
 * What this file does:
 * - Follows a polyline path defined by a list of waypoints in the robot's odometry/world frame.
 * - Each control cycle, it picks a "lookahead" target point that is ~LOOKAHEAD_DIST ahead on the path.
 * - It converts that target into walking commands using a pose-to-pose controller:
 *   robotPu.walk(walkSpeed, walkTurn)
 *
 * How to use:
 * - Edit `waypoints` to your desired path points.
 * - Tune `LOOKAHEAD_DIST` (bigger => smoother, smaller => tighter tracking).
 * - Tune `ARRIVE_DIST` (distance threshold to advance to the next waypoint).
 *
 * Coordinate & angle conventions (must match robotPu.locationArray()):
 * - x: left/right
 * - y: forward/back
 * - theta returned by the robot is in degrees; this file converts it to radians.
 * - Internally, this file uses the standard right-handed convention:
 *   - theta = 0 points along +X
 *   - positive theta is CCW toward +Y
 *   - bearing to a point is atan2(dy, dx)
 * - robotPu.locationArray()[2] uses a +Y-forward convention (0 means +Y), so we convert it.
 *
 * Notes:
 * - If your robot turns the wrong direction, the sign of `kTurn` in `updateControl()` may need flipping.
 */

interface Pose2D {
    x: number
    y: number
    theta: number
}

function clamp(x: number, lo: number, hi: number): number {
    return Math.max(lo, Math.min(hi, x))
}

function wrapPi(a: number): number {
    while (a > Math.PI) a -= 2 * Math.PI
    while (a <= -Math.PI) a += 2 * Math.PI
    return a
}

function norm2(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

function ppDeg2rad(deg: number): number {
    return (deg * Math.PI) / 180
}

// Coordinate conventions (match robotPu.locationArray()):
// - x: left/right
// - y: forward/back
// - Internal theta: heading measured from +x (standard math convention)
// Therefore:
// - heading unit vector = [cos(theta), sin(theta)]
// - bearing to a point is atan2(dy, dx)

function thetaStdFromRobotDeg(thetaRobotDeg: number): number {
    // RobotPU: 0 deg is +Y. Standard: 0 rad is +X.
    // If robot points +Y, standard heading should be +90 deg.
    return ppDeg2rad(90 - thetaRobotDeg)
}

function updateControl(
    current: Pose2D,
    target: Pose2D,
    offsetStartDist: number = 50,
    stopDist: number = 40
): number[] {
    const vMax = 2.5
    const turnMax = 0.8
    const kTurn = -2.0

    const leadMin = 0.05
    const leadMax = 0.18
    const lateralOffsetMax = 0.10

    const dx = target.x - current.x
    const dy = target.y - current.y
    const dist = norm2(dx, dy)

    if (dist < stopDist) {
        return [0, 0]
    }

    const tx = Math.cos(target.theta)
    const ty = Math.sin(target.theta)
    const nx = -ty
    const ny = tx

    const vx = current.x - target.x
    const vy = current.y - target.y
    const cross = tx * vy - ty * vx
    const side = cross >= 0 ? -1 : 1

    const offsetGain = clamp(1.0 - dist / offsetStartDist, 0.0, 1.0)
    const lead = leadMin + (leadMax - leadMin) * offsetGain
    const lateral = (lateralOffsetMax * offsetGain) * side

    const xV = target.x + lead * tx + lateral * nx
    const yV = target.y + lead * ty + lateral * ny

    const headingToV = Math.atan2(yV - current.y, xV - current.x)
    const eHeading = wrapPi(headingToV - current.theta)

    const walkTurn = clamp(kTurn * eHeading, -turnMax, turnMax)

    let walkSpeed = vMax
    if (Math.abs(walkTurn) > 0.9 * turnMax) {
        walkSpeed *= 0.6
    }

    return [walkSpeed, walkTurn]
}

const LOOKAHEAD_DIST = 120
const ARRIVE_DIST = 60

const waypoints: { x: number, y: number }[] = [
    { x: 0, y: 200 },
    { x: 100, y: 400 },
    { x: -200, y: 600 },
]

let wpIdx = 0
let controlVec: number[] = [0, 0]

// End-of-path safety: if we are trying to reach the final waypoint but keep getting farther
// away (off-track / diverging), stop rather than chase forever.
let lastWpBestDist = 1e9
let lastWpWorsenCount = 0

function segmentHeading(from: { x: number, y: number }, to: { x: number, y: number }): number {
    return Math.atan2(to.y - from.y, to.x - from.x)
}

function lookaheadTarget(cur: { x: number, y: number }, idx: number, lookahead: number): Pose2D {
    // Choose a target pose that is `lookahead` distance ahead along the polyline.
    //
    // Intuition:
    // - Think of the path as a chain of straight segments between waypoints.
    // - Starting from the current position, we "walk" along those segments, consuming
    //   the remaining lookahead distance until we land inside one segment.
    // - The returned theta is the path tangent (heading) at that target point.
    let x = cur.x
    let y = cur.y
    let remain = lookahead
    let i = idx

    while (i < waypoints.length) {
        const wx = waypoints[i].x
        const wy = waypoints[i].y
        const dx = wx - x
        const dy = wy - y
        const d = norm2(dx, dy)

        if (d < 1e-6) {
            // Degenerate segment (current point already at the next waypoint):
            // skip it to avoid divide-by-zero.
            i += 1
            continue
        }

        if (d >= remain) {
            // The target lies within the current segment (x,y) -> (wx,wy).
            // Interpolate by fraction t = remain / segment_length.
            const t = remain / d
            const xt = x + dx * t
            const yt = y + dy * t
            // Path tangent heading at this point.
            const theta = Math.atan2(dy, dx)
            return { x: xt, y: yt, theta: theta }
        }

        // Otherwise the lookahead target is beyond this waypoint.
        // Consume this whole segment and continue to the next.
        remain -= d
        x = wx
        y = wy
        i += 1
    }

    // If we ran past the end of the path, clamp target at the last waypoint.
    // Heading is set to the last segment direction (prev -> last) so the robot
    // approaches the end with a meaningful final orientation.
    const last = waypoints[waypoints.length - 1]
    const prev = waypoints.length >= 2 ? waypoints[waypoints.length - 2] : last
    return { x: last.x, y: last.y, theta: segmentHeading(prev, last) }
}

robotPu.setServoTrim(-5, 0, -5, 0, -8, 0)

basic.forever(function () {
    const loc = robotPu.locationArray()
    const curPose: Pose2D = { x: loc[0], y: loc[1], theta: thetaStdFromRobotDeg(loc[2]) }

    if (wpIdx < waypoints.length) {
        // Advance waypoint if we are close enough OR if we have passed it along the path direction.
        // This prevents getting stuck if the robot drifts off track and never enters the ARRIVE_DIST circle.
        const wx = waypoints[wpIdx].x
        const wy = waypoints[wpIdx].y

        const dx = wx - curPose.x
        const dy = wy - curPose.y
        const dist = norm2(dx, dy)

        // Standard "arrive" condition.
        let advance = dist < ARRIVE_DIST

        // "Passed" condition (only meaningful if there is a next waypoint to define the segment direction).
        // Segment: waypoint[wpIdx] -> waypoint[wpIdx + 1]
        // If dot( robot - waypoint[wpIdx], segment_dir ) > 0 then robot is in front of the waypoint along that segment.
        if (!advance && wpIdx + 1 < waypoints.length) {
            const nx = waypoints[wpIdx + 1].x
            const ny = waypoints[wpIdx + 1].y
            const sx = nx - wx
            const sy = ny - wy
            const segLen2 = sx * sx + sy * sy
            if (segLen2 > 1e-6) {
                const rx = curPose.x - wx
                const ry = curPose.y - wy
                const dot = rx * sx + ry * sy
                advance = dot > 0
            }
        }

        // Special-case: last waypoint.
        // If we overshoot the final waypoint but never enter the ARRIVE_DIST circle,
        // we still want to stop. Use the direction of the final segment (prev -> last)
        // and check if the robot is in front of the last waypoint along that direction.
        if (!advance && wpIdx === waypoints.length - 1 && waypoints.length >= 2) {
            const prev = waypoints[wpIdx - 1]
            const sx = wx - prev.x
            const sy = wy - prev.y
            const segLen2 = sx * sx + sy * sy
            if (segLen2 > 1e-6) {
                const rx = curPose.x - wx
                const ry = curPose.y - wy
                const dot = rx * sx + ry * sy
                advance = dot > 0
            }
        }

        // Additional last-waypoint safety: if we are diverging from the final waypoint,
        // stop after a short grace period.
        if (!advance && wpIdx === waypoints.length - 1) {
            if (dist < lastWpBestDist) {
                lastWpBestDist = dist
                lastWpWorsenCount = 0
            } else {
                lastWpWorsenCount += 1
            }

            // If distance hasn't improved for ~2 seconds (100 * 20ms) and we're still far,
            // consider the run done to avoid endless chasing.
            if (lastWpWorsenCount > 100 && dist > 2 * ARRIVE_DIST) {
                advance = true
            }
        }

        if (advance) {
            wpIdx += 1

            // Reset end-of-path safety when we switch waypoints.
            lastWpBestDist = 1e9
            lastWpWorsenCount = 0
        }
    }

    if (wpIdx >= waypoints.length) {
        controlVec = [0, 0]
        basic.pause(20)
        return
    }

    const target = lookaheadTarget({ x: curPose.x, y: curPose.y }, wpIdx, LOOKAHEAD_DIST)
    controlVec = updateControl(curPose, target)

    serial.writeLine("wpIdx:" + wpIdx)
    serial.writeLine("x:" + curPose.x)
    serial.writeLine("y:" + curPose.y)
    serial.writeLine("theta(rad):" + curPose.theta)
    serial.writeLine("tx:" + target.x)
    serial.writeLine("ty:" + target.y)

    basic.pause(20)
})

basic.forever(function () {
    robotPu.walk(controlVec[0], controlVec[1])
    basic.pause(10)
})

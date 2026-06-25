/**
 * RobotPU soccer: viewpoint-to-viewpoint controller (pose -> pose).
 *
 * Goal:
 * - Drive from current pose (x, y, theta) to a target pose (x, y, theta)
 * - Prefer fast, smooth arcs over slow in-place turning (biped gait constraint)
 * - Arrive near the target already aligned with the desired final heading
 *
 * Output:
 * - updateControl(current, target) returns [walkSpeed, walkTurn] for robotPu.walk(...)
 *
 * Key idea ("virtual target" with lateral offset):
 * - Instead of aiming directly at the target position, we aim at a point that is:
 *   - slightly "ahead" of the target along target.theta (lead distance), and
 *   - slightly offset left/right of the target heading line (lateral offset)
 * - This creates an approach arc that naturally pre-turns the robot into the final heading,
 *   leaving less work at the final kick pose.
 *
 * Usage pattern:
 * - Call updateControl(...) every control cycle (e.g., 20 ms)
 * - Feed the returned commands to robotPu.walk(...)
 *
 * Units:
 * - x, y are in the same units as robotPu.locationArray() provides
 * - theta is in radians (must match robotPu.locationArray() convention)
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

function signNonZero(x: number): number {
    return x >= 0 ? 1 : -1
}

// function deg2rad(deg: number): number {
//     return (deg * Math.PI) / 180
// }

// Reusable pose-to-pose controller.
// Return [walkSpeed, walkTurn] compatible with robotPu.walk(walkSpeed, walkTurn).
//
// Key idea: aim at a virtual target point that is:
// - slightly past the target pose along its desired final heading ("lead"), and
// - slightly offset left/right ("lateral offset")
// so the robot naturally arcs into the final heading without a slow in-place turn.
// Start shaping the approach when we are within this distance of the target.
// Stop when we are close enough.
function updateControl(current: Pose2D, target: Pose2D,
    offsetStartDist: number = 50,
    stopDist: number = 40): number[] {
    // --- Tuning knobs (start values; tune on the field) ---
    const vMax = 2.5
    const turnMax = 0.8
    const kTurn = -2.0 // if robot turn wrong direction, flip the subtraction order

    // Virtual-target geometry.
    const leadMin = 0.05
    const leadMax = 0.18
    const lateralOffsetMax = 0.10

    const dx = target.x - current.x
    const dy = target.y - current.y
    const dist = norm2(dx, dy)

    if (dist < stopDist) {
        // Close enough: stop. (Caller can switch to a different state, e.g. kick/alignment.)
        return [0, 0]
    }

    // Unit vectors of the desired final heading.
    // RobotPU odometry uses a +Y-forward convention, so heading is measured from +Y.
    const tx = Math.sin(target.theta)
    const ty = Math.cos(target.theta)
    const nx = -ty
    const ny = tx

    // Which side of the final-heading line are we currently on?
    // v = current - target
    const vx = current.x - target.x
    const vy = current.y - target.y
    const cross = tx * vy - ty * vx

    // Choose an offset side. Using the opposite side creates "room" for the approach arc.
    const side = -signNonZero(cross)

    // Ramp in the offset when mid-close.
    // - Far away: offsetGain ~ 0, so we mostly aim straight toward the target.
    // - Near the target: offsetGain -> 1, so we apply more lead/lateral shaping.
    const offsetGain = clamp(1.0 - dist / offsetStartDist, 0.0, 1.0)
    const lead = leadMin + (leadMax - leadMin) * offsetGain
    const lateral = (lateralOffsetMax * offsetGain) * side

    // Virtual target point.
    // This point is placed slightly ahead of (and slightly to the side of) the true target.
    // Following it tends to create an arc that finishes aligned with target.theta.
    const xV = target.x + lead * tx + lateral * nx
    const yV = target.y + lead * ty + lateral * ny

    // Steer toward the virtual target.
    // RobotPU convention in this repo: heading/bearing is atan2(dx, dy) (note dx first).
    const headingToV = Math.atan2(xV - current.x, yV - current.y)
    const eHeading = wrapPi(headingToV -current.theta) 

    const walkTurn = clamp(kTurn * eHeading, -turnMax, turnMax)

    // Run fast, but reduce speed a bit if we are saturating turn (curvature limit).
    // Intuition: if turn is near max, slow down to avoid unstable gait / oversteer.
    let walkSpeed = vMax
    if (Math.abs(walkTurn) > 0.9 * turnMax) {
        walkSpeed *= 0.6
    }

    return [walkSpeed, walkTurn]
}


robotPu.setServoTrim(-5, 0, -5, 0, -8, 0)

// Target pose (x, y, theta in degrees)
// Note: theta must be in radians. Convert degrees if needed.
let dstPose: Pose2D = { x: 0, y: 400, theta: Math.PI / 2 }
let controlVec: number[] = [2, 0] // start with forward speed > 0
let locArr: number[] = robotPu.locationArray()
let lastLoc = locArr.slice()
// read current pose and compute controls
basic.forever(function () {
    // read currrent pose
    locArr = robotPu.locationArray()
    // only compute control when location change, this require robot moves first.
    if (locArr[0] != lastLoc[0] || locArr[1] != lastLoc[1] || locArr[2] != lastLoc[2]) {
        lastLoc = locArr.slice()
        let neckYaw = 90 - robotPu.ServoTargets()[4] // compute neck yaw. 90 degree is neutral
        // neck yaw is added to theta from body frame, using head frame
        let curPose: Pose2D = { x: locArr[0], y: locArr[1], theta: deg2rad(locArr[2] - neckYaw) }
        // compute controls
        controlVec = updateControl(curPose, dstPose)
        // print current pose
        serial.writeLine("x:" + curPose.x)
        serial.writeLine("y:" + curPose.y)
        serial.writeLine("theta in rad:" + curPose.theta)
    }
    basic.pause(20)
})

// take actions
basic.forever(function () {
    robotPu.walk(controlVec[0], controlVec[1]) 
    basic.pause(10)
})
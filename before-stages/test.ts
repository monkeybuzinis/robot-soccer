/**
 * test.ts — MVP-first build, following the EXACT dependency order from
 * robotpu_soccer.pdf (Chapter 1, section 5; expanded in 03_localmap.md):
 *
 *   1. I2C read and packet validity (VALID/STALE)
 *   2. Ball position in robot frame (head yaw/pitch + camera offset)
 *   3. 2D projection + mapping into the 10x10 grid
 *   4. Kick target computation (point behind ball along ball-to-goal line)
 *   5. Navigation (simple steering first, then obstacle avoidance)
 *
 * Built from the files YOU provided (not robotpu-soccer-final.ts /
 * robotpu-detect-ball-goal.ts):
 *   - robotpu-i2c-cam.ts       -> packet layout constants, i8/i16/u16, flagsText()
 *   - robotpu-soccer-mvp.ts    -> trackBall() head-tracking shape, VALID/STALE gating,
 *                                 boot sequence, service-enable loop, actuator loop
 *   - robotpu-localmap.ts      -> LocalGrid class, computeKickPoint(), desiredHeadingTo(),
 *                                 and the SIMPLE if/else steering controller (this file
 *                                 intentionally does NOT use A* (robotpu-A-star.ts),
 *                                 Kalman filtering (robotpu-kalman-filter.ts), the
 *                                 virtual-target controller (robotpu-viewpoint.ts), or
 *                                 odometry latency-compensation -- those are Chapter 4/5
 *                                 refinements that come AFTER this MVP order, not part of it)
 *   - robotpu-search-soccer.js -> SEARCH_PATTERN head-scan sweep
 *
 * Two corrections carried over from robotpu-soccer-final.ts (confirmed bugs against
 * the real https://github.com/robotgyms/pxt-robotpu source -- NOT stylistic changes,
 * the original files will not run correctly without these):
 *   - Namespace is `robotPuPro`, not `robotPu`; `servoTargets()` is lowercase.
 *   - Head yaw/pitch servo targets are ABSOLUTE degrees in [0,179] with 90=neutral
 *     (not a small offset clamped to [-45,45], which pins the head at one extreme).
 *
 * NEW in this file -- step 2 of the dependency order, MISSING from every provided
 * file: a real camera-frame -> robot-frame transform. robotpu-soccer-mvp.ts and
 * robotpu-localmap.ts both store x_mm/y_mm directly as if it were already a
 * robot-frame ground point. That's only true when the head is pointed dead ahead;
 * as soon as the head yaws to track an off-center ball, the same camera reading
 * means a different robot-frame position. See cameraToRobotFrame() below and PDF
 * Chapter 1 section 3.2 ("Transformations").
 *
 * Explicitly OUT of scope for this MVP file (per the dependency order above):
 *   - Odometry-based latency compensation (Chapter 5) -- ball/goal positions are
 *     computed fresh from the current head angle each packet, no time-travel.
 *   - Kalman smoothing (Chapter 5) -- raw transformed positions are used directly.
 *   - A* planning (Chapter 4) -- navigation is direct heading-based steering only.
 *   - Real obstacle avoidance -- no sonar packet format exists in any provided
 *     file; left as a marked TODO (milestone: "SLAM-inspired local mapping").
 *
 * IMPORTANT -- MakeCode project setup: this file is self-contained and meant to
 * be the ONLY .ts file in the MakeCode project (same caveat as the other files
 * in this folder: they all declare the same global names).
 */

// ---------------------------------------------------------------------------
// STEP 1 (a): I2C protocol / packet layout (robotpu-i2c-cam.ts)
// ---------------------------------------------------------------------------
const MUX_ADDR = 112 // 0x70 (I2C multiplexer)
const ESP32_ADDR = 66 // 0x42 (camera coprocessor)
const SIZE = 18 // bytes per detection packet

const SOCCER_BALL = 4
const SOCCER_GOAL = 5

const CMD_SERVICE_ENABLE = 8
const SERVICE_WIFI = 1
const SERVICE_IMAGE_CAPTURE = 2
const SERVICE_FACE_DETECTION = 3
const SERVICE_SOCCER_BALL_DETECTION = 4
const SERVICE_SOCCER_GOAL_DETECTION = 5

const VALID = 1 << 0
const STALE = 1 << 1
const CAPTURE = 1 << 2
const WEB = 1 << 3
const SLEEP = 1 << 4

// ---------------------------------------------------------------------------
// Tuning (robotpu-soccer-mvp.ts / robotpu-localmap.ts)
// ---------------------------------------------------------------------------
const SCAN_WAIT_FRAMES = 25
const LOST_TIMEOUT_MS = 6000
const DEBUG_FLAG = true

// Head servo range: pxt-robotpu's PCB.servoStep() clamps to an ABSOLUTE [0,179]
// degree range, with 90 = looking straight ahead (confirmed against the real
// extension source, not present in the originally provided files).
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45

// Local planning grid (Chapter 3, robotpu-localmap.ts): 10x10 cells, 0.05 m/cell.
const GRID_N = 10
const GRID_RES_M = 0.05
const GRID_HALF_M = (GRID_N * GRID_RES_M) / 2

// Kick geometry / simple-steering tuning (robotpu-localmap.ts).
const KICK_BACKOFF_M = 0.05
const APPROACH_SLOW_M = 0.25
const KICK_DIST_M = 0.11
const TURN_GAIN = -1.2 // flip sign if the robot turns the wrong way

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------
function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

function norm2L(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

// 2D rotation by theta (radians).
function rot(theta: number, x: number, y: number): number[] {
    const c = Math.cos(theta)
    const s = Math.sin(theta)
    return [c * x - s * y, s * x + c * y]
}

// Parse a signed byte from the camera packet.
function i8(v: number): number {
    return v >= 128 ? v - 256 : v
}

function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}

function u16(buf: Buffer, offset: number): number {
    return buf[offset] | (buf[offset + 1] << 8)
}

function flagsText(f: number): string {
    let s = ""
    if (f & VALID) s += " valid"
    if (f & STALE) s += " stale"
    if (f & CAPTURE) s += " capture"
    if (f & WEB) s += " web"
    if (f & SLEEP) s += " sleep"
    return s.length > 0 ? s.trim() : "none"
}

// ---------------------------------------------------------------------------
// STEP 3: Local occupancy/measurement grid (robotpu-localmap.ts's LocalGrid)
// ---------------------------------------------------------------------------
class LocalGrid {
    public g: number[][]
    constructor() {
        this.g = []
        for (let i = 0; i < GRID_N; i++) {
            const row: number[] = []
            for (let j = 0; j < GRID_N; j++) row.push(0)
            this.g.push(row)
        }
    }
    clear() {
        for (let i = 0; i < GRID_N; i++)
            for (let j = 0; j < GRID_N; j++) this.g[i][j] = 0
    }
    inBounds(i: number, j: number): boolean {
        return i >= 0 && i < GRID_N && j >= 0 && j < GRID_N
    }
    index(x_m: number, y_m: number): number[] {
        const j = Math.floor((x_m + GRID_HALF_M) / GRID_RES_M)
        const i = Math.floor(y_m / GRID_RES_M)
        return [i, j]
    }
    set(i: number, j: number, v: number) {
        if (this.inBounds(i, j)) this.g[i][j] = v
    }
    get(i: number, j: number): number {
        return this.inBounds(i, j) ? this.g[i][j] : 0
    }
}

const grid = new LocalGrid()

// ---------------------------------------------------------------------------
// STEP 2 (NEW): camera frame -> robot frame transform.
//
// Camera mount offset in the robot base frame, per PDF Chapter 1 section 3.2
// and the comment in robotpu-search-soccer.js ("camera is put at (0, 35, 160)
// millimeters at robot frame"): x=0 (centered left/right), y=0.035m (35mm
// forward of body origin), z=0.160m (160mm up -- unused here, we project
// straight to the 2D ground plane like the original files do).
const CAM_OFFSET_X_M = 0
const CAM_OFFSET_Y_M = 0.035

// Flip back to +1 if THIS sign turns out wrong too -- "increasing HeadYaw servo
// angle = turning which way" could not be independently verified from the
// extension source alone (same kind of field-tunable sign flag as TURN_GAIN
// elsewhere in this project). Started at +1; flipped to -1 after a report of
// the robot walking toward the wrong spot, which is the classic symptom of
// this rotation being applied with the wrong sign.
const HEAD_YAW_SIGN = -1

// Rotate a camera-frame ground point by the camera's current pan (head yaw)
// to express it in the robot BODY frame, then add the fixed camera offset.
// headYawServoDeg is the ABSOLUTE current head yaw servo angle (90 = centered).
function cameraToRobotFrame(camX_m: number, camY_m: number, headYawServoDeg: number): number[] {
    const headYawDeg = HEAD_YAW_SIGN * (headYawServoDeg - HEAD_YAW_CENTER)
    const headYawRad = headYawDeg * Math.PI / 180
    const bodyXY = rot(headYawRad, camX_m, camY_m)
    if (DEBUG_FLAG) {
        serial.writeLine(`xform: cam=(${camX_m},${camY_m}) headYawServo=${headYawServoDeg} headYawDeg=${headYawDeg} -> body=(${bodyXY[0] + CAM_OFFSET_X_M},${bodyXY[1] + CAM_OFFSET_Y_M})`)
    }
    return [bodyXY[0] + CAM_OFFSET_X_M, bodyXY[1] + CAM_OFFSET_Y_M]
}

// ---------------------------------------------------------------------------
// STEP 4: Kick target computation (robotpu-localmap.ts)
// ---------------------------------------------------------------------------
// Kick point is behind the ball along the direction from goal -> ball.
function computeKickPoint(ball_now: number[], goal_now: number[]): number[] {
    const dx = ball_now[0] - goal_now[0]
    const dy = ball_now[1] - goal_now[1]
    const n = Math.max(1e-6, norm2L(dx, dy))
    const ux = dx / n
    const uy = dy / n
    return [ball_now[0] + ux * KICK_BACKOFF_M, ball_now[1] + uy * KICK_BACKOFF_M]
}

// Face the goal while standing at kick point: desired heading is from robot to goal.
function desiredHeadingTo(x: number, y: number): number {
    return Math.atan2(x, y)
}

// ---------------------------------------------------------------------------
// Camera service helpers (robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

function setSoccerDetection(enabled: boolean) {
    setService(SERVICE_SOCCER_BALL_DETECTION, enabled)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, enabled)
}

// ---------------------------------------------------------------------------
// State (robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
let scanStepIndex = 0
let scanFrameCounter = 0

let currentPitch = 0
let currentYaw = 0

let yaw = 0
let pitch = 0
let lastBallSeenTime = 0
let lastGoalSeenTime = 0
let search_gain = 1
let walkSpeed = 0
let walkTurn = 0
let walkMode = 0

// Ball/goal positions, already transformed into the robot frame (step 2).
// No separate "_cam2D"/"_pose_O" bookkeeping: this MVP does not do odometry
// latency compensation (that's a Chapter 5 refinement, out of scope here).
let ball_now: number[] = [0, 0]
let ball_valid = false
let ball_rx_ms = 0

let goal_now: number[] = [0, 0]
let goal_valid = false
let goal_rx_ms = 0

const SEARCH_PATTERN: { y: number, p: number }[] = [
    { y: 15, p: 0 },
    { y: -15, p: 0 },
    { y: -15, p: -10 },
    { y: 0, p: -10 },
    { y: 15, p: -10 },
    { y: 15, p: 3 },
    { y: 0, p: 3 },
    { y: -15, p: 3 },
    { y: -15, p: 0 },
    { y: 0, p: 0 }
]

// ---------------------------------------------------------------------------
// Head-scanning search pattern (robotpu-search-soccer.js)
// ---------------------------------------------------------------------------
function searchBall(searchPattern: { y: number, p: number }[]) {
    yaw *= 0.5
    pitch *= 0.5

    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = searchPattern[scanStepIndex]
        if (DEBUG_FLAG) {
            serial.writeLine("" + (`yawSearch: ${targetOffset.y * search_gain}`))
            serial.writeLine("" + (`pitchSearch: ${targetOffset.p * search_gain}`))
        }
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const nextYaw = clampL(currentYaw + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(currentPitch + targetOffset.p * search_gain, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 1)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 1)
        currentYaw = nextYaw
        currentPitch = nextPitch
        robotPuPro.leftEyeBright(0.002)
        robotPuPro.rightEyeBright(0.002)
        return
    }

    scanFrameCounter = SCAN_WAIT_FRAMES
    scanStepIndex += 1
    if (scanStepIndex >= SEARCH_PATTERN.length) {
        scanStepIndex = 0
        search_gain = Math.min(4, search_gain * 1.1)
    }
}

// ---------------------------------------------------------------------------
// STEP 1 (b) + STEP 2: packet validity gating, head-tracking, and the new
// camera-frame -> robot-frame transform.
// ---------------------------------------------------------------------------
function trackBall(p: Buffer) {
    const currentTime = input.runningTime()

    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    let type = p[0]
    let flags = p[3]
    let count = p[4]
    let score = p[5]

    if (DEBUG_FLAG) {
        serial.writeLine(`pkt type=${type} flags=${flagsText(flags)} count=${count} score=${score}`)
    }

    // STEP 1: packet validity (VALID/STALE gating).
    if (!(flags & VALID)) count = 0

    if (type == SOCCER_BALL) {
        if (count > 0) {
            lastBallSeenTime = currentTime
            search_gain = 1.0
            let x_mm = i16(p, 6)
            let y_mm = i16(p, 8)
            yaw = i8(p[16])
            pitch = i8(p[17])
            const staleScale = (flags & STALE) ? 0.3 : 1.0
            const yawCmd = yaw * staleScale
            const pitchCmd = pitch * staleScale

            // Move head to keep the ball centered (closed-loop visual servoing).
            robotPuPro.setModeVar(robotPuPro.Mode.API)
            const nextYaw = clampL(currentYaw + yawCmd * 0.08, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(currentPitch + pitchCmd * 0.08, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)
            currentYaw = nextYaw
            currentPitch = nextPitch
            robotPuPro.leftEyeBright(0.01)
            robotPuPro.rightEyeBright(0.01)

            // STEP 2: only fresh (non-stale) measurements get the robot-frame transform.
            if (!(flags & STALE)) {
                ball_now = cameraToRobotFrame(x_mm / 1000, y_mm / 1000, currentYaw)
                ball_valid = true
                ball_rx_ms = currentTime
            }
        } else if (currentTime - lastBallSeenTime < LOST_TIMEOUT_MS) {
            // Briefly follow through in the last known direction.
            yaw *= 0.7
            pitch *= 0.7
            const nextYaw = clampL(currentYaw + yaw * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(currentPitch + pitch * 0.2, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 5)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 5)
            currentYaw = nextYaw
            currentPitch = nextPitch
        } else {
            searchBall(SEARCH_PATTERN)
        }
    } else if (type == SOCCER_GOAL) {
        if (count > 0) {
            lastGoalSeenTime = currentTime
            let x_mm = i16(p, 6)
            let y_mm = i16(p, 8)

            // STEP 2: same transform, using whatever the head's current yaw is
            // right now (it's normally tracking the ball, not the goal -- the
            // transform is about where the CAMERA is pointed, not the target).
            if (!(flags & STALE)) {
                goal_now = cameraToRobotFrame(x_mm / 1000, y_mm / 1000, currentYaw)
                goal_valid = true
                goal_rx_ms = currentTime
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Boot sequence (robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
robotPuPro.setChannel(166)
robotPuPro.setServoTrim(-5, 0, -5, 0, -8, 0)

radio.onReceivedString(function (receivedString: string) {
    robotPuPro.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name: string, value: number) {
    robotPuPro.runKeyValueCommand(name, value)
})

input.onButtonPressed(Button.A, function () {
    setService(SERVICE_WIFI, true)
})
input.onButtonPressed(Button.B, function () {
    setService(SERVICE_WIFI, false)
})
input.onLogoEvent(TouchButtonEvent.Pressed, function () {
    robotPuPro.toggleServoTrim()
    basic.pause(500)
})

basic.showString("I")
pins.i2cWriteNumber(MUX_ADDR, 15, NumberFormat.Int8LE, false)
// wait camera boots up
basic.pause(2000)

// handle camera reboot: by default, all services are off when ESP32 reboots
basic.forever(function () {
    setService(SERVICE_IMAGE_CAPTURE, true)
    basic.pause(10)
    setSoccerDetection(true)
    basic.pause(10)
    setService(SERVICE_FACE_DETECTION, false)
    basic.pause(10)
    if (DEBUG_FLAG) {
        setService(SERVICE_WIFI, true)
    } else {
        setService(SERVICE_WIFI, false)
    }
    basic.pause(30000)
})

currentYaw = robotPuPro.servoTargets()[4]
currentPitch = robotPuPro.servoTargets()[5]

// ---------------------------------------------------------------------------
// STEP 3 + STEP 4 + STEP 5: mapping, kick target, simple steering.
// Ported from robotpu-localmap.ts's if/else planner almost verbatim -- the
// only change is that ball_now/goal_now now arrive already in the robot frame
// (via cameraToRobotFrame() in trackBall() above) instead of raw camera mm.
// ---------------------------------------------------------------------------
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackBall(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }

    const now_ms = input.runningTime()
    if (ball_valid && (now_ms - ball_rx_ms) > LOST_TIMEOUT_MS) ball_valid = false
    if (goal_valid && (now_ms - goal_rx_ms) > LOST_TIMEOUT_MS) goal_valid = false

    // STEP 3: 2D projection + mapping into the 10x10 grid.
    // 0 = empty, 2 = ball, 3 = goal, 4 = kick point
    grid.clear()
    if (ball_valid) {
        const idxB = grid.index(ball_now[0], ball_now[1])
        grid.set(idxB[0], idxB[1], 2)
    }
    if (goal_valid) {
        const idxG = grid.index(goal_now[0], goal_now[1])
        grid.set(idxG[0], idxG[1], 3)
    }

    // STEP 4 + STEP 5: kick target, then simple heading-based steering
    // (TODO obstacle avoidance: no sonar packet format exists yet in any
    // provided file -- this is the "SLAM-inspired" milestone item, still to
    // come; for now the grid above only tracks ball/goal, not obstacles).
    if (ball_valid && goal_valid) {
        const kickPt = computeKickPoint(ball_now, goal_now)
        const idxK = grid.index(kickPt[0], kickPt[1])
        grid.set(idxK[0], idxK[1], 4)

        const distKick = norm2L(kickPt[0], kickPt[1])
        const headingKick = desiredHeadingTo(kickPt[0], kickPt[1])
        walkTurn = clampL(TURN_GAIN * headingKick, -0.8, 0.8)

        // Approach kick point
        if (distKick > KICK_DIST_M) {
            walkSpeed = distKick > APPROACH_SLOW_M ? 2.5 : 1.5
            if (DEBUG_FLAG) {
                serial.writeLine(`speed: ${walkSpeed}`)
                serial.writeLine(`turn: ${walkTurn}`)
            }
            walkMode = 0
        } else {
            // At kick point: face goal then kick
            const headingGoal = desiredHeadingTo(goal_now[0], goal_now[1])
            walkTurn = clampL(TURN_GAIN * headingGoal, -0.8, 0.8)
            if (Math.abs(headingGoal) > 0.25) {
                walkSpeed = 1.0
                walkMode = 1 // align: back up + turn so we don't drift into the ball
            } else {
                walkSpeed = 0
                walkMode = 2 // kick
            }
        }
    } else if (ball_valid) {
        // Fallback: ball only. Approach ball with a simple local controller.
        // (This matches robotpu-localmap.ts's original behavior. If you find the
        // robot ends up on the wrong side of the ball to kick toward the goal,
        // see robotpu-soccer-final.ts's actionHoldForGoal()/btHoldForGoal() for
        // the alternative "wait for both before moving" strategy.)
        const distBall = norm2L(ball_now[0], ball_now[1])
        const headingBall = desiredHeadingTo(ball_now[0], ball_now[1])
        walkTurn = clampL(TURN_GAIN * headingBall, -0.8, 0.8)
        walkSpeed = distBall > 0.25 ? 2.0 : 1.0
        walkMode = 0
    } else {
        walkTurn = 0
        walkSpeed = 0
        walkMode = 0
        searchBall(SEARCH_PATTERN)
    }

    if (DEBUG_FLAG && (ball_valid || goal_valid)) {
        if (ball_valid) serial.writeLine(`ball_now x=${ball_now[0]} y=${ball_now[1]}`)
        if (goal_valid) serial.writeLine(`goal_now x=${goal_now[0]} y=${goal_now[1]}`)
    }
    basic.pause(20)
})

// ---------------------------------------------------------------------------
// Actuator loop (robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
basic.forever(function () {
    if (walkMode == 0) {
        robotPuPro.walk(walkSpeed, walkTurn)
    } else if (walkMode == 1) {
        robotPuPro.walk(-walkSpeed, walkTurn)
    } else if (walkMode == 2) {
        robotPuPro.kick()
    }
    basic.pause(10)
})

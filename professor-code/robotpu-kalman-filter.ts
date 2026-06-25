
/**
 * RobotPU soccer: Kalman filtered local 2D map + planning in the camera frame.
 *
 * This file is based on `robotpu-soccer-mvp.ts`, but adds a lightweight Kalman filter
 * to smooth `ball_now` and `goal_now` after latency compensation.
 */

declare const robotPu: any
declare const radio: any
declare const input: any
declare const pins: any
declare const basic: any
declare const serial: any
declare const IconNames: any
declare const NumberFormat: any
declare const Button: any
declare const TouchButtonEvent: any
declare const Buffer: any
declare function updateControl(current: any, target: any, offsetStartDist?: number, stopDist?: number): number[]

function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

function norm2L(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

// 2D rotation
function rot(theta: number, x: number, y: number): number[] {
    const c = Math.cos(theta)
    const s = Math.sin(theta)
    return [c * x - s * y, s * x + c * y]
}

// Parse Unsigned Char
function i8(v: number) {
    return v >= 128 ? v - 256 : v
}

class Kalman1DConstVel {
    public x0: number
    public x1: number
    public P00: number
    public P01: number
    public P10: number
    public P11: number

    constructor() {
        this.x0 = 0
        this.x1 = 0
        this.P00 = 1
        this.P01 = 0
        this.P10 = 0
        this.P11 = 1
    }

    reset(pos: number) {
        this.x0 = pos
        this.x1 = 0
        this.P00 = 0.05
        this.P01 = 0
        this.P10 = 0
        this.P11 = 1
    }

    predict(dt_s: number, q_pos: number, q_vel: number) {
        const dt = Math.max(0, dt_s)

        // State prediction: [p; v] <- [1 dt; 0 1] [p; v]
        this.x0 = this.x0 + dt * this.x1

        // Covariance prediction: P <- F P F^T + Q
        const P00 = this.P00
        const P01 = this.P01
        const P10 = this.P10
        const P11 = this.P11

        // F = [[1, dt],[0,1]]
        // FPF^T gives:
        // P00' = P00 + dt(P10 + P01) + dt^2 P11
        // P01' = P01 + dt P11
        // P10' = P10 + dt P11
        // P11' = P11
        this.P00 = P00 + dt * (P10 + P01) + dt * dt * P11 + q_pos
        this.P01 = P01 + dt * P11
        this.P10 = P10 + dt * P11
        this.P11 = P11 + q_vel
    }

    update(z: number, r: number) {
        const S = this.P00 + r
        if (S <= 1e-9) return

        // Innovation
        const y = z - this.x0

        // Kalman gain (H=[1,0])
        const K0 = this.P00 / S
        const K1 = this.P10 / S

        // State update
        this.x0 = this.x0 + K0 * y
        this.x1 = this.x1 + K1 * y

        // Covariance update: P <- (I - K H) P
        const P00 = this.P00
        const P01 = this.P01
        const P10 = this.P10
        const P11 = this.P11

        const IK0 = 1 - K0
        this.P00 = IK0 * P00
        this.P01 = IK0 * P01
        this.P10 = P10 - K1 * P00
        this.P11 = P11 - K1 * P01
    }
}

class Kalman2DConstVel {
    public kx: Kalman1DConstVel
    public ky: Kalman1DConstVel
    public inited: boolean

    constructor() {
        this.kx = new Kalman1DConstVel()
        this.ky = new Kalman1DConstVel()
        this.inited = false
    }

    reset(x: number, y: number) {
        this.kx.reset(x)
        this.ky.reset(y)
        this.inited = true
    }

    predict(dt_s: number, q_pos: number, q_vel: number) {
        if (!this.inited) return
        this.kx.predict(dt_s, q_pos, q_vel)
        this.ky.predict(dt_s, q_pos, q_vel)
    }

    update(meas_x: number, meas_y: number, r: number) {
        if (!this.inited) {
            this.reset(meas_x, meas_y)
            return
        }
        this.kx.update(meas_x, r)
        this.ky.update(meas_y, r)
    }

    pos(): number[] {
        return [this.kx.x0, this.ky.x0]
    }
}

const MUX_ADDR = 112 // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

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

const SCAN_WAIT_FRAMES = 25
const LOST_TIMEOUT_MS = 6000
const DEBUG_FLAG = true

// Grid map (local, in current robot/camera frame)
const GRID_N = 10
const GRID_RES_M = 0.05
const GRID_HALF_M = (GRID_N * GRID_RES_M) / 2

// Simple planner/controller tuning
const KICK_BACKOFF_M = 0.05
const APPROACH_SLOW_M = 0.25
const KICK_DIST_M = 0.11
const TURN_GAIN = -1.2 // flip the sign if robot turns in wrong direction

// Kalman tuning
// Goal is stationary: almost constant position.
const GOAL_Q_POS = 1e-6
const GOAL_Q_VEL = 1e-4
const GOAL_R_FRESH = 8e-4
const GOAL_R_STALE = 5e-3

// Ball is stationary before kick, more dynamic after kick.
const BALL_Q_POS_PRE = 5e-6
const BALL_Q_VEL_PRE = 5e-4
const BALL_Q_POS_POST = 5e-4
const BALL_Q_VEL_POST = 5e-2
const BALL_R_FRESH = 1.5e-3
const BALL_R_STALE = 8e-3

// After a kick, allow the ball filter to adapt quickly.
const BALL_POSTKICK_MS = 1500

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

let lastKickMs = -999999

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

robotPu.setChannel(166)
// set your servo trim here for better walking control
robotPu.setServoTrim(-5, 0, -5, 0, -8, 0)
radio.onReceivedString(function (receivedString: string) {
    robotPu.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name: string, value: number) {
    robotPu.runKeyValueCommand(name, value)
})

input.onButtonPressed(Button.A, function () {
    setService(SERVICE_WIFI, true)
})
input.onButtonPressed(Button.B, function () {
    setService(SERVICE_WIFI, false)
})
input.onLogoEvent(TouchButtonEvent.Pressed, function () {
    // allow gamepad to trim servos to improve balancing
    robotPu.toggleServoTrim()
    basic.pause(500)
})

function i16(buf: any, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}
function u16(buf: any, offset: number): number {
    return buf[offset] | (buf[offset + 1] << 8)
}

function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

function setSoccerDetection(enabled: boolean) {
    setService(SERVICE_SOCCER_BALL_DETECTION, enabled)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, enabled)
}

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
        robotPu.setModeVar(robotPu.Mode.API)
        const nextYaw = clampL(currentYaw + targetOffset.y * search_gain, -45, 45)
        const nextPitch = clampL(currentPitch + targetOffset.p * search_gain, -45, 45)
        robotPu.servoStep(robotPu.ServoJoint.HeadYaw, nextYaw, 1)
        robotPu.servoStep(robotPu.ServoJoint.HeadPitch, nextPitch, 1)
        currentYaw = nextYaw
        currentPitch = nextPitch
        robotPu.leftEyeBright(0.002)
        robotPu.rightEyeBright(0.002)
        return
    }

    scanFrameCounter = SCAN_WAIT_FRAMES
    scanStepIndex += 1
    if (scanStepIndex >= SEARCH_PATTERN.length) {
        scanStepIndex = 0
        search_gain = Math.min(4, search_gain * 1.1)
    }
}

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
        for (let i = 0; i < GRID_N; i++) {
            for (let j = 0; j < GRID_N; j++) this.g[i][j] = 0
        }
    }
    inBounds(i: number, j: number): boolean {
        return i >= 0 && i < GRID_N && j >= 0 && j < GRID_N
    }
    index(x_m: number, y_m: number): number[] {
        const j = Math.floor((x_m + GRID_HALF_M) / GRID_RES_M)
        const i = Math.floor((y_m) / GRID_RES_M)
        return [i, j]
    }
    set(i: number, j: number, v: number) {
        if (this.inBounds(i, j)) this.g[i][j] = v
    }
}

const grid = new LocalGrid()

// return robot current pose in odometry frame
function getPoseO(): number[] {
    const loc = robotPu.locationArray()
    return [loc[0], loc[1], loc[2]]
}

// Transform a 2D point measured in robot/camera-local frame at time t_cam into
// the current robot-local frame using odometry.
function camToNow(cam2D_at_det: number[], pose_det_O: number[], pose_now_O: number[]): number[] {
    const det_xy_O = rot(pose_det_O[2], cam2D_at_det[0], cam2D_at_det[1])
    const obj_O = [pose_det_O[0] + det_xy_O[0], pose_det_O[1] + det_xy_O[1]]

    const rel_O = [obj_O[0] - pose_now_O[0], obj_O[1] - pose_now_O[1]]
    const rel_now = rot(-pose_now_O[2], rel_O[0], rel_O[1])
    return rel_now
}

function computeKickPoint(ball_now: number[], goal_now: number[]): number[] {
    const dx = ball_now[0] - goal_now[0]
    const dy = ball_now[1] - goal_now[1]
    const n = Math.max(1e-6, norm2L(dx, dy))
    const ux = dx / n
    const uy = dy / n
    return [ball_now[0] + ux * KICK_BACKOFF_M, ball_now[1] + uy * KICK_BACKOFF_M]
}

function desiredHeadingTo(x: number, y: number): number {
    return Math.atan2(x, y)
}

// Detection state stored at image time (t_cam):
let ball_cam2D: number[] = [0, 0]
let ball_pose_O: number[] = [0, 0, 0]
let ball_valid = false
let ball_rx_ms = 0

let goal_cam2D: number[] = [0, 0]
let goal_pose_O: number[] = [0, 0, 0]
let goal_valid = false
let goal_rx_ms = 0

const ballKF = new Kalman2DConstVel()
const goalKF = new Kalman2DConstVel()
let lastKfMs = -1

function trackBall(p: any) {
    const currentTime = input.runningTime()

    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    let type = p[0]
    let flags = p[3]
    let count = p[4]

    if (!(flags & VALID)) {
        count = 0
    }

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

            robotPu.setModeVar(robotPu.Mode.API)
            const nextYaw = clampL(currentYaw + yawCmd * 0.08, -45, 45)
            const nextPitch = clampL(currentPitch + pitchCmd * 0.08, -45, 45)
            robotPu.servoStep(robotPu.ServoJoint.HeadYaw, nextYaw, 8)
            robotPu.servoStep(robotPu.ServoJoint.HeadPitch, nextPitch, 8)
            currentYaw = nextYaw
            currentPitch = nextPitch
            robotPu.leftEyeBright(0.01)
            robotPu.rightEyeBright(0.01)

            // Save the ball measurement (ground-plane projection) for mapping/planning.
            if (!(flags & STALE)) {
                const poseO = getPoseO()
                ball_cam2D = [x_mm / 1000, y_mm / 1000]
                ball_pose_O = [poseO[0], poseO[1], poseO[2]]
                ball_valid = true
                ball_rx_ms = currentTime
            }
        } else if (currentTime - lastBallSeenTime < LOST_TIMEOUT_MS) {
            yaw *= 0.7
            pitch *= 0.7
            const nextYaw = clampL(currentYaw + yaw * 0.2, -45, 45)
            const nextPitch = clampL(currentPitch + pitch * 0.2, -45, 45)
            robotPu.servoStep(robotPu.ServoJoint.HeadYaw, nextYaw, 5)
            robotPu.servoStep(robotPu.ServoJoint.HeadPitch, nextPitch, 5)
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

            if (!(flags & STALE)) {
                const poseO = getPoseO()
                goal_cam2D = [x_mm / 1000, y_mm / 1000]
                goal_pose_O = [poseO[0], poseO[1], poseO[2]]
                goal_valid = true
                goal_rx_ms = currentTime
            }
        }
    }
}

basic.showString("I")
pins.i2cWriteNumber(
    MUX_ADDR,
    15,
    NumberFormat.Int8LE,
    false
)
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

currentYaw = robotPu.ServoTargets()[4]
currentPitch = robotPu.ServoTargets()[5]

basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackBall(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }

    const now_ms = input.runningTime()
    const pose_now_O = getPoseO()

    // Latency handling: if a measurement is too old, drop it.
    if (ball_valid && (now_ms - ball_rx_ms) > LOST_TIMEOUT_MS) ball_valid = false
    if (goal_valid && (now_ms - goal_rx_ms) > LOST_TIMEOUT_MS) goal_valid = false

    // Transform detections into the current local frame.
    let ball_meas: number[] = [0, 0]
    let goal_meas: number[] = [0, 0]

    const haveBallMeas = ball_valid
    const haveGoalMeas = goal_valid

    if (haveBallMeas) {
        ball_meas = camToNow(ball_cam2D, ball_pose_O, pose_now_O)
    }
    if (haveGoalMeas) {
        goal_meas = camToNow(goal_cam2D, goal_pose_O, pose_now_O)
    }

    // Kalman predict step
    if (lastKfMs < 0) lastKfMs = now_ms
    const dt_s = Math.min(0.2, Math.max(0, (now_ms - lastKfMs) / 1000))
    lastKfMs = now_ms

    // Goal: stationary
    goalKF.predict(dt_s, GOAL_Q_POS, GOAL_Q_VEL)

    // Ball: switch noise after kick
    const postKick = (now_ms - lastKickMs) < BALL_POSTKICK_MS
    const bq_pos = postKick ? BALL_Q_POS_POST : BALL_Q_POS_PRE
    const bq_vel = postKick ? BALL_Q_VEL_POST : BALL_Q_VEL_PRE
    ballKF.predict(dt_s, bq_pos, bq_vel)

    // Kalman measurement update (fresh measurements only; STALE stored measurements are never written into *_cam2D)
    if (haveGoalMeas) {
        goalKF.update(goal_meas[0], goal_meas[1], GOAL_R_FRESH)
    }
    if (haveBallMeas) {
        ballKF.update(ball_meas[0], ball_meas[1], BALL_R_FRESH)
    }

    // Use filtered positions if initialized; else fall back to measurements
    let ball_now: number[] = haveBallMeas ? ball_meas : [0, 0]
    let goal_now: number[] = haveGoalMeas ? goal_meas : [0, 0]

    if (ballKF.inited) ball_now = ballKF.pos()
    if (goalKF.inited) goal_now = goalKF.pos()

    // Build a small local grid for debugging/visualization.
    grid.clear()
    if (ball_valid) {
        const idxB = grid.index(ball_now[0], ball_now[1])
        grid.set(idxB[0], idxB[1], 2)
    }
    if (goal_valid) {
        const idxG = grid.index(goal_now[0], goal_now[1])
        grid.set(idxG[0], idxG[1], 3)
    }

    if (ball_valid && goal_valid) {
        const kickPt = computeKickPoint(ball_now, goal_now)
        const idxK = grid.index(kickPt[0], kickPt[1])
        grid.set(idxK[0], idxK[1], 4)

        const distKick = norm2L(kickPt[0], kickPt[1])
        const thetaKick = desiredHeadingTo(goal_now[0] - kickPt[0], goal_now[1] - kickPt[1])

        if (distKick > KICK_DIST_M) {
            const ctrl = updateControl(
                { x: 0, y: 0, theta: 0 },
                { x: kickPt[0], y: kickPt[1], theta: thetaKick },
                0.25,
                KICK_DIST_M
            )
            walkSpeed = ctrl[0]
            walkTurn = ctrl[1]
            walkMode = 0
        } else {
            const headingGoal = desiredHeadingTo(goal_now[0], goal_now[1])
            walkTurn = clampL(TURN_GAIN * headingGoal, -0.8, 0.8)
            if (Math.abs(headingGoal) > 0.25) {
                walkSpeed = 1.0
                walkMode = 1
            } else {
                walkSpeed = 0
                walkMode = 2
            }
        }
    } else if (ball_valid) {
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

basic.forever(function () {
    if (walkMode == 0) {
        robotPu.walk(walkSpeed, walkTurn)
    } else if (walkMode == 1) {
        robotPu.walk(-walkSpeed, walkTurn)
    } else if (walkMode == 2) {
        lastKickMs = input.runningTime()
        robotPu.kick()
    }
    basic.pause(10)
})

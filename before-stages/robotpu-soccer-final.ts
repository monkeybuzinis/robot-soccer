/**
 * RobotPU Soccer — FINAL integrated program
 * ==========================================
 * Implements the full pipeline described in robotpu_soccer.pdf (Chapters 2-7):
 *
 *   1) Identify ball & goal   -> ESP32-S3 camera over I2C (trackBall())
 *   2) Localize detections    -> odometry-based latency compensation (camToNow())
 *   3) Smooth estimates       -> per-axis constant-velocity Kalman filters (ballKF/goalKF)
 *   4) Compute kick position  -> point behind the ball, on the goal->ball line (computeKickPoint())
 *   5) Plan a path            -> A* on a 10x10 local grid, replanned every cycle (astarGrid())
 *   6) Navigate + orient      -> heading-aware pose-to-pose controller (updateControl())
 *   7) Decide what to do      -> Behavior Tree (btRoot()) replacing a flat if/else planner
 *   8) Act                    -> robotPuPro.walk(...) / robotPuPro.kick() in the actuator loop
 *
 * Grading-rubric coverage (see Chapter 1.1):
 *   - Follow ball:                  head tracking in trackBall() (the body holds position via
 *     actionHoldForGoal() until the goal is also visible -- see "Strategy" note below)
 *   - Navigate to kick position:    actionApproachKickPoint() + astarGrid() replanning
 *   - Contact ball with goal orientation: actionAlignToGoal()
 *   - Push ball toward goal:        actionKick() (repeats every cycle until scored)
 *   - Score detection:              condScored()/btCelebrate() -- position-based heuristic
 *     (no ball-entered-goal sensor exists in the packet format; see SCORE_DIST_M)
 *   - Obstacle avoidance (alpha):    condPathClear()/actionReplan() treat ball+goal cells as
 *     occupied in the planning grid (no sonar wiring yet — see README "Known limitations")
 *
 * Strategy (detect-both-then-plan, not chase-then-hope):
 *   The robot deliberately does NOT walk toward the ball just because the ball alone is
 *   visible. Walking toward the ball without knowing the goal's position can leave the
 *   robot on the wrong side of the ball to ever kick it goal-ward. So btRoot()'s fallback
 *   order is: btScore() (needs BOTH ball+goal) -> btHoldForGoal() (ball seen, body holds
 *   still, head keeps tracking the ball while waiting for the goal) -> actionSearchBallBT()
 *   (nothing seen, head sweeps to reacquire). Only once both are valid does any walkSpeed/
 *   walkTurn ever get set to a nonzero value.
 *
 * IMPORTANT — MakeCode project setup:
 *   This file is self-contained and is meant to be the ONLY .ts file in the MakeCode project.
 *   Do not add it alongside robotpu-soccer-mvp.ts / robotpu-kalman-filter.ts / robotpu-localmap.ts /
 *   robotpu-search-soccer.js / robotpu-i2c-cam.ts — they declare the same global consts/functions
 *   (SIZE, ESP32_ADDR, trackBall, LocalGrid, ...) and MakeCode merges all .ts files into one
 *   global scope, which will cause "duplicate identifier" build errors.
 *
 * Coordinate conventions (Chapter 1, section 3.1/3.2):
 *   - All planning happens in the CURRENT local frame: the robot is always at (0,0), theta=0.
 *   - x = left(-)/right(+), y = forward(+)/backward(-), matching robotPuPro.locationArray().
 *   - Internal heading convention: theta=0 along +Y (forward), bearing = atan2(dx, dy).
 */

// ---------------------------------------------------------------------------
// 1) I2C protocol / packet layout (ESP32-S3 vision coprocessor)
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
// 2) Tunable constants
// ---------------------------------------------------------------------------
const SCAN_WAIT_FRAMES = 25
const LOST_TIMEOUT_MS = 6000
const DEBUG_FLAG = true

// Head servo range (pxt-robotpu PCB.servoStep clamps to absolute [0,179] degrees;
// 90 is "looking straight ahead" for both yaw and pitch -- see RobotPu.joystick()'s
// `90 + headYawBias` / `90 + headPitchBias`). currentYaw/currentPitch below are
// ABSOLUTE servo angles, not offsets from 0, so clamp around this center.
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45

// Local planning grid (Chapter 3): 10x10 cells, 0.05 m/cell, robot-centered.
const GRID_N = 10
const GRID_RES_M = 0.05
const GRID_HALF_M = (GRID_N * GRID_RES_M) / 2

// Kick geometry / controller tuning (Chapter 6 section 5).
const KICK_BACKOFF_M = 0.05
const KICK_DIST_M = 0.11
const TURN_GAIN = -1.2 // flip sign if the robot turns the wrong way
const ALIGN_HEADING_TOL = 0.25 // ~14 deg

// "Scored" heuristic: the packet format has no explicit ball-entered-goal sensor
// event, so we approximate it from position -- once the ball's estimate is within
// this distance of the goal center it's considered in/at the goal. The goal mouth
// is ~0.297m wide (A4 width, Chapter 1 game rules) and the ball is 0.05m across,
// so a threshold a bit under the goal half-width avoids false positives from the
// ball merely rolling past on its way toward the kick point.
const SCORE_DIST_M = 0.12

// Kalman tuning (Chapter 5 section 9): goal is stationary, ball changes after kick.
const GOAL_Q_POS = 1e-6
const GOAL_Q_VEL = 1e-4
const GOAL_R_FRESH = 8e-4
const GOAL_R_STALE = 5e-3

const BALL_Q_POS_PRE = 5e-6
const BALL_Q_VEL_PRE = 5e-4
const BALL_Q_POS_POST = 5e-4
const BALL_Q_VEL_POST = 5e-2
const BALL_R_FRESH = 1.5e-3
const BALL_R_STALE = 8e-3
const BALL_POSTKICK_MS = 1500

// Behavior Tree return codes (Chapter 7).
const BT_RUNNING = 0
const BT_FAILURE = 1
const BT_SUCCESS = 2

// ---------------------------------------------------------------------------
// 3) Math helpers
// ---------------------------------------------------------------------------
function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

function norm2L(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

function wrapPiL(a: number): number {
    while (a > Math.PI) a -= 2 * Math.PI
    while (a < -Math.PI) a += 2 * Math.PI
    return a
}

function signNonZero(x: number): number {
    return x >= 0 ? 1 : -1
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

interface Pose2D {
    x: number
    y: number
    theta: number
}

// ---------------------------------------------------------------------------
// 4) Local occupancy/measurement grid (Chapter 3): robot-centered, current frame.
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
    // x_m: left(-)/right(+); y_m: forward distance from the robot.
    index(x_m: number, y_m: number): number[] {
        const j = Math.floor((x_m + GRID_HALF_M) / GRID_RES_M)
        const i = Math.floor(y_m / GRID_RES_M)
        return [i, j]
    }
    // Inverse of index(): metric coordinates of a cell's center.
    center(i: number, j: number): number[] {
        const x_m = (j + 0.5) * GRID_RES_M - GRID_HALF_M
        const y_m = (i + 0.5) * GRID_RES_M
        return [x_m, y_m]
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
// 5) A* planner on the local grid (Chapter 4 / Chapter 6 section 4).
//    4-connected, Manhattan heuristic, returns a path of linear cell indices
//    (i*GRID_N+j), or [] if no path exists.
// ---------------------------------------------------------------------------
function astarGrid(si: number, sj: number, gi: number, gj: number, occ: LocalGrid): number[] {
    if (!occ.inBounds(si, sj) || !occ.inBounds(gi, gj)) return []

    const n = GRID_N * GRID_N
    const startKey = si * GRID_N + sj
    const goalKey = gi * GRID_N + gj

    const gScore: number[] = []
    const fScore: number[] = []
    const cameFrom: number[] = []
    const closed: boolean[] = []
    const inOpen: boolean[] = []
    const open: number[] = []

    for (let k = 0; k < n; k++) {
        gScore.push(1e9)
        fScore.push(1e9)
        cameFrom.push(-1)
        closed.push(false)
        inOpen.push(false)
    }

    gScore[startKey] = 0
    fScore[startKey] = Math.abs(si - gi) + Math.abs(sj - gj)
    open.push(startKey)
    inOpen[startKey] = true

    while (open.length > 0) {
        let bestIdx = 0
        let bestKey = open[0]
        let bestF = fScore[bestKey]
        for (let t = 1; t < open.length; t++) {
            const k = open[t]
            if (fScore[k] < bestF) {
                bestF = fScore[k]
                bestKey = k
                bestIdx = t
            }
        }
        open.removeAt(bestIdx)
        inOpen[bestKey] = false

        if (bestKey === goalKey) {
            const path: number[] = []
            let cur = goalKey
            while (cur !== -1) {
                path.push(cur)
                if (cur === startKey) break
                cur = cameFrom[cur]
            }
            path.reverse()
            return path
        }

        closed[bestKey] = true
        const ci = Math.idiv(bestKey, GRID_N)
        const cj = bestKey - ci * GRID_N

        const neighI = [ci - 1, ci + 1, ci, ci]
        const neighJ = [cj, cj, cj - 1, cj + 1]

        for (let u = 0; u < 4; u++) {
            const ni = neighI[u]
            const nj = neighJ[u]
            if (!occ.inBounds(ni, nj)) continue
            if (occ.get(ni, nj) >= 1) continue // occupied (obstacle / ball / goal cell)

            const nk = ni * GRID_N + nj
            if (closed[nk]) continue

            const tentativeG = gScore[bestKey] + 1
            if (tentativeG < gScore[nk]) {
                cameFrom[nk] = bestKey
                gScore[nk] = tentativeG
                fScore[nk] = tentativeG + Math.abs(ni - gi) + Math.abs(nj - gj)
                if (!inOpen[nk]) {
                    open.push(nk)
                    inOpen[nk] = true
                }
            }
        }
    }
    return []
}

// ---------------------------------------------------------------------------
// 6) Kalman filters (Chapter 5): per-axis constant-velocity 1D filters,
//    combined into a 2D filter for ball/goal position smoothing.
// ---------------------------------------------------------------------------
class Kalman1DConstVel {
    public x0: number // position
    public x1: number // velocity
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
        this.x0 = this.x0 + dt * this.x1

        const P00 = this.P00
        const P01 = this.P01
        const P10 = this.P10
        const P11 = this.P11

        // F = [[1, dt],[0,1]]; P <- F P F^T + Q
        this.P00 = P00 + dt * (P10 + P01) + dt * dt * P11 + q_pos
        this.P01 = P01 + dt * P11
        this.P10 = P10 + dt * P11
        this.P11 = P11 + q_vel
    }

    update(z: number, r: number) {
        const S = this.P00 + r
        if (S <= 1e-9) return

        const y = z - this.x0
        const K0 = this.P00 / S
        const K1 = this.P10 / S

        this.x0 = this.x0 + K0 * y
        this.x1 = this.x1 + K1 * y

        const P00 = this.P00
        const P01 = this.P01
        const P11 = this.P11

        const IK0 = 1 - K0
        this.P00 = IK0 * P00
        this.P01 = IK0 * P01
        this.P10 = this.P10 - K1 * P00
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

const ballKF = new Kalman2DConstVel()
const goalKF = new Kalman2DConstVel()

// ---------------------------------------------------------------------------
// 7) Global state
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
let walkMode = 0 // 0=walk fwd, 1=back up+turn (align), 2=kick

let lastKickMs = -999999
let lastKfMs = -1
let scored = false // set once the ball is judged to have reached the goal (see SCORE_DIST_M)
let celebrated = false // guards actionCelebrate() so the "Goal!" announcement fires only once

// Detection state stored at image time (t_cam):
// - *_cam2D is a 2D point in the camera/robot-local frame at capture time (meters)
// - *_pose_O is the robot odometry pose at receipt time (used for latency compensation)
let ball_cam2D: number[] = [0, 0]
let ball_pose_O: number[] = [0, 0, 0]
let ball_valid = false
let ball_rx_ms = 0

let goal_cam2D: number[] = [0, 0]
let goal_pose_O: number[] = [0, 0, 0]
let goal_valid = false
let goal_rx_ms = 0

// Latency-compensated + Kalman-filtered estimates in the CURRENT local frame.
let ball_now: number[] = [0, 0]
let goal_now: number[] = [0, 0]

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
// 8) Camera service helpers (I2C control plane)
// ---------------------------------------------------------------------------
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
// 9) Pose / frame transforms (Chapter 3)
// ---------------------------------------------------------------------------

// Robot pose in the odometry frame, from robotPuPro.locationArray().
// NOTE: the real API returns [x_mm, y_mm, theta_deg] (millimeters + degrees,
// +Y-forward convention) -- NOT meters/radians. Convert here so every other
// function in this file (rot(), camToNow(), desiredHeadingTo(), updateControl())
// can consistently assume meters + radians.
function getPoseO(): number[] {
    const loc = robotPuPro.locationArray()
    return [loc[0] / 1000, loc[1] / 1000, loc[2] * Math.PI / 180]
}

// Transform a 2D point measured at time t_cam (in the robot-local frame at that
// time, pose_det_O) into the CURRENT robot-local frame (pose_now_O), using odometry
// to compensate for the robot's motion since the detection (Chapter 3.2 / Chapter 5).
function camToNow(cam2D_at_det: number[], pose_det_O: number[], pose_now_O: number[]): number[] {
    const det_xy_O = rot(pose_det_O[2], cam2D_at_det[0], cam2D_at_det[1])
    const obj_O = [pose_det_O[0] + det_xy_O[0], pose_det_O[1] + det_xy_O[1]]

    const rel_O = [obj_O[0] - pose_now_O[0], obj_O[1] - pose_now_O[1]]
    const rel_now = rot(-pose_now_O[2], rel_O[0], rel_O[1])
    return rel_now
}

// Kick point: KICK_BACKOFF_M behind the ball, along the goal->ball direction,
// so pushing the ball from there sends it straight at the goal.
function computeKickPoint(ball_xy: number[], goal_xy: number[]): number[] {
    const dx = ball_xy[0] - goal_xy[0]
    const dy = ball_xy[1] - goal_xy[1]
    const n = Math.max(1e-6, norm2L(dx, dy))
    const ux = dx / n
    const uy = dy / n
    return [ball_xy[0] + ux * KICK_BACKOFF_M, ball_xy[1] + uy * KICK_BACKOFF_M]
}

// Bearing to a point in the +Y-forward convention used by robotPuPro.locationArray().
function desiredHeadingTo(x: number, y: number): number {
    return Math.atan2(x, y)
}

// ---------------------------------------------------------------------------
// 10) Pose-to-pose controller (Chapter 4 / Chapter 6 section 5): "virtual target"
//     with lead + lateral offset so the robot arcs into the final heading instead
//     of doing a slow in-place turn (better suited to a biped gait).
// ---------------------------------------------------------------------------
function updateControl(current: Pose2D, target: Pose2D,
    offsetStartDist: number = 0.25, stopDist: number = KICK_DIST_M): number[] {
    const vMax = 2.5
    const turnMax = 0.8
    const kTurn = -2.0 // flip sign if the robot turns the wrong way

    const leadMin = 0.05
    const leadMax = 0.18
    const lateralOffsetMax = 0.10

    const dx = target.x - current.x
    const dy = target.y - current.y
    const dist = norm2L(dx, dy)

    if (dist < stopDist) return [0, 0]

    const tx = Math.sin(target.theta)
    const ty = Math.cos(target.theta)
    const nx = -ty
    const ny = tx

    const vx = current.x - target.x
    const vy = current.y - target.y
    const cross = tx * vy - ty * vx
    const side = -signNonZero(cross)

    const offsetGain = clampL(1.0 - dist / offsetStartDist, 0.0, 1.0)
    const lead = leadMin + (leadMax - leadMin) * offsetGain
    const lateral = (lateralOffsetMax * offsetGain) * side

    const xV = target.x + lead * tx + lateral * nx
    const yV = target.y + lead * ty + lateral * ny

    const headingToV = Math.atan2(xV - current.x, yV - current.y)
    const eHeading = wrapPiL(headingToV - current.theta)

    const cmdTurn = clampL(kTurn * eHeading, -turnMax, turnMax)

    let cmdSpeed = vMax
    if (Math.abs(cmdTurn) > 0.9 * turnMax) cmdSpeed *= 0.6

    return [cmdSpeed, cmdTurn]
}

// ---------------------------------------------------------------------------
// 11) Camera packet handling: identify ball & goal, drive head tracking,
//     and store raw measurements + odometry pose for latency compensation.
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

            // Only store FRESH (non-stale) measurements for mapping/planning.
            if (!(flags & STALE)) {
                const poseO = getPoseO()
                ball_cam2D = [x_mm / 1000, y_mm / 1000]
                ball_pose_O = [poseO[0], poseO[1], poseO[2]]
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

// ---------------------------------------------------------------------------
// 12) Behavior Tree (Chapter 7): replaces the flat if/else planner.
//
//   Root (Fallback)
//     Celebrate (Sequence)            -- checked first; overrides everything once latched
//       COND scored
//       ACTION celebrate
//     Score (Sequence)
//       COND ballVisible / goalVisible
//       btNavigateToKick (Fallback: direct approach, else A* replan)
//       ACTION alignToGoal
//       ACTION kick
//     HoldForGoal (Sequence)
//       COND ballVisible
//       ACTION holdForGoal               -- body holds still, head still tracks ball
//     ACTION searchBallBT
// ---------------------------------------------------------------------------

function btSequence(children: (() => number)[]): number {
    for (let i = 0; i < children.length; i++) {
        const result = children[i]()
        if (result !== BT_SUCCESS) return result // FAILURE or RUNNING stops here
    }
    return BT_SUCCESS
}

function btFallback(children: (() => number)[]): number {
    for (let i = 0; i < children.length; i++) {
        const result = children[i]()
        if (result !== BT_FAILURE) return result // SUCCESS or RUNNING stops here
    }
    return BT_FAILURE
}

// --- Condition nodes (instant, no side effects) ---

function condBallVisible(): number {
    const age = input.runningTime() - ball_rx_ms
    return (ball_valid && age < LOST_TIMEOUT_MS) ? BT_SUCCESS : BT_FAILURE
}

function condGoalVisible(): number {
    const age = input.runningTime() - goal_rx_ms
    return (goal_valid && age < LOST_TIMEOUT_MS) ? BT_SUCCESS : BT_FAILURE
}

// Is the straight-line path to the kick point obstacle-free?
// Samples 3 waypoints between the robot (0,0) and the kick point on `grid`
// (the same grid the main loop fills with ball/goal/kick markers each cycle).
function condPathClear(): number {
    if (!ball_valid || !goal_valid) return BT_FAILURE
    const kickPt = computeKickPoint(ball_now, goal_now)
    for (let t = 1; t <= 3; t++) {
        const fx = kickPt[0] * t / 4
        const fy = kickPt[1] * t / 4
        const cell = grid.index(fx, fy)
        if (grid.get(cell[0], cell[1]) >= 1) return BT_FAILURE
    }
    return BT_SUCCESS
}

function condAtKickPoint(): number {
    if (!ball_valid || !goal_valid) return BT_FAILURE
    const kickPt = computeKickPoint(ball_now, goal_now)
    return norm2L(kickPt[0], kickPt[1]) <= KICK_DIST_M ? BT_SUCCESS : BT_FAILURE
}

function condAlignedToGoal(): number {
    if (!goal_valid) return BT_FAILURE
    const heading = desiredHeadingTo(goal_now[0], goal_now[1])
    return Math.abs(heading) <= ALIGN_HEADING_TOL ? BT_SUCCESS : BT_FAILURE
}

// Has the ball reached the goal? Position-based heuristic (see SCORE_DIST_M);
// once true it stays true (scored is a one-way latch -- there's no "ball left
// the goal" un-scoring in this demo).
function condScored(): number {
    if (scored) return BT_SUCCESS
    if (!ball_valid || !goal_valid) return BT_FAILURE
    const d = norm2L(ball_now[0] - goal_now[0], ball_now[1] - goal_now[1])
    if (d < SCORE_DIST_M) {
        scored = true
        return BT_SUCCESS
    }
    return BT_FAILURE
}

// --- Action nodes (may return RUNNING across ticks; set walkSpeed/walkTurn/walkMode) ---

// Walk straight toward the kick point. RUNNING until close enough.
function actionApproachKickPoint(): number {
    if (!ball_valid || !goal_valid) return BT_FAILURE
    const kickPt = computeKickPoint(ball_now, goal_now)
    const dist = norm2L(kickPt[0], kickPt[1])
    if (dist <= KICK_DIST_M) return BT_SUCCESS

    const thetaKick = desiredHeadingTo(goal_now[0] - kickPt[0], goal_now[1] - kickPt[1])
    const ctrl = updateControl(
        { x: 0, y: 0, theta: 0 },
        { x: kickPt[0], y: kickPt[1], theta: thetaKick },
        0.25,
        KICK_DIST_M
    )
    walkSpeed = ctrl[0]
    walkTurn = ctrl[1]
    walkMode = 0
    return BT_RUNNING
}

// Rotate in place to face the goal. RUNNING until aligned.
function actionAlignToGoal(): number {
    if (!goal_valid) return BT_FAILURE
    const heading = desiredHeadingTo(goal_now[0], goal_now[1])
    if (Math.abs(heading) > ALIGN_HEADING_TOL) {
        walkTurn = clampL(TURN_GAIN * heading, -0.8, 0.8)
        walkSpeed = 1.0
        walkMode = 1 // back-up-and-turn mode (avoids drifting into the ball while aligning)
        return BT_RUNNING
    }
    walkSpeed = 0
    walkTurn = 0
    walkMode = 0
    return BT_SUCCESS
}

// Execute the kick / push action. Sets walkMode=2 for one cycle, then SUCCESS.
function actionKick(): number {
    walkSpeed = 0
    walkTurn = 0
    walkMode = 2 // picked up by the actuator loop -> robotPuPro.kick()
    return BT_SUCCESS
}

// Celebration once condScored() latches true. Stops all walking/kicking --
// there's nothing left to do once the ball is in. The "Goal!" announcement
// itself only fires once (celebrated guard); after that this just holds
// position quietly every tick.
function actionCelebrate(): number {
    walkSpeed = 0
    walkTurn = 0
    walkMode = 0
    if (!celebrated) {
        celebrated = true
        if (DEBUG_FLAG) serial.writeLine("SCORED!")
        robotPuPro.talk("Goal!")
    }
    return BT_SUCCESS
}

// Hold position while the ball is visible but the goal is not yet.
// We deliberately do NOT walk toward the ball here: approaching it without
// knowing where the goal is can leave the robot on the wrong side of the
// ball to ever kick it toward the goal. The head keeps tracking the ball
// (already handled per-packet in trackBall()); the body just waits until
// both ball_valid && goal_valid before btScore() takes over and plans the
// kick-point approach.
function actionHoldForGoal(): number {
    if (!ball_valid) return BT_FAILURE
    walkSpeed = 0
    walkTurn = 0
    walkMode = 0
    return BT_RUNNING
}

// Sweep the head to find the ball (nothing seen at all).
function actionSearchBallBT(): number {
    walkSpeed = 0
    walkTurn = 0
    walkMode = 0
    searchBall(SEARCH_PATTERN)
    return BT_RUNNING // always running; ball/goal detections break out via the Fallback above it
}

// Replan via A* when the direct line to the kick point is blocked.
// Marks the ball/goal cells as obstacles, finds a path, and steers toward
// the first waypoint of that path (re-planned every cycle).
function actionReplan(): number {
    if (!ball_valid || !goal_valid) return BT_FAILURE
    const kickPt = computeKickPoint(ball_now, goal_now)

    const occ = new LocalGrid()
    occ.clear()
    const b = occ.index(ball_now[0], ball_now[1])
    occ.set(b[0], b[1], 1)
    const g = occ.index(goal_now[0], goal_now[1])
    occ.set(g[0], g[1], 1)

    const start = occ.index(0, 0)
    const goalCell = occ.index(kickPt[0], kickPt[1])
    const path = astarGrid(start[0], start[1], goalCell[0], goalCell[1], occ)

    if (path.length < 2) return BT_FAILURE // no path found

    // Step toward the first waypoint along the found path.
    const next = path[1]
    const ni = Math.idiv(next, GRID_N)
    const nj = next - ni * GRID_N
    const wp = occ.center(ni, nj)

    const thetaKick = desiredHeadingTo(goal_now[0] - kickPt[0], goal_now[1] - kickPt[1])
    const ctrl = updateControl(
        { x: 0, y: 0, theta: 0 },
        { x: wp[0], y: wp[1], theta: thetaKick },
        0.50,
        KICK_DIST_M
    )
    walkSpeed = ctrl[0]
    walkTurn = ctrl[1]
    walkMode = 0
    return BT_RUNNING
}

// Navigate-or-replan sub-tree: try the direct approach first; if the path is
// blocked, fall back to A* replanning.
function btNavigateToKick(): number {
    return btFallback([
        () => btSequence([condPathClear, actionApproachKickPoint]),
        actionReplan
    ])
}

// Ball already judged in the goal: stop everything, announce once, hold forever.
// Checked first in btRoot() so it overrides every other branch once it latches.
function btCelebrate(): number {
    return btSequence([condScored, actionCelebrate])
}

// Full score sequence: see ball + goal, navigate, align, kick.
function btScore(): number {
    return btSequence([
        condBallVisible,
        condGoalVisible,
        btNavigateToKick,
        actionAlignToGoal,
        actionKick
    ])
}

// Ball seen, goal not yet: hold position (see actionHoldForGoal() above for why).
function btHoldForGoal(): number {
    return btSequence([
        condBallVisible,
        actionHoldForGoal
    ])
}

// Root: celebrate if already scored, else try to score, else hold position
// waiting for the goal, else search.
function btRoot(): number {
    return btFallback([
        btCelebrate,
        btScore,
        btHoldForGoal,
        actionSearchBallBT
    ])
}

// ---------------------------------------------------------------------------
// 13) Boot sequence
// ---------------------------------------------------------------------------
robotPuPro.setChannel(166)
// set your servo trim here for better walking control
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
    // allow gamepad to trim servos to improve balancing
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
// 14) Main perception + mapping + planning loop
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
    const pose_now_O = getPoseO()

    // Drop measurements that are too old.
    if (ball_valid && (now_ms - ball_rx_ms) > LOST_TIMEOUT_MS) ball_valid = false
    if (goal_valid && (now_ms - goal_rx_ms) > LOST_TIMEOUT_MS) goal_valid = false

    // Latency compensation: transform stored detections into the current frame.
    let ball_meas: number[] = [0, 0]
    let goal_meas: number[] = [0, 0]
    if (ball_valid) ball_meas = camToNow(ball_cam2D, ball_pose_O, pose_now_O)
    if (goal_valid) goal_meas = camToNow(goal_cam2D, goal_pose_O, pose_now_O)

    // Kalman predict step.
    if (lastKfMs < 0) lastKfMs = now_ms
    const dt_s = Math.min(0.2, Math.max(0, (now_ms - lastKfMs) / 1000))
    lastKfMs = now_ms

    goalKF.predict(dt_s, GOAL_Q_POS, GOAL_Q_VEL) // goal is stationary

    const postKick = (now_ms - lastKickMs) < BALL_POSTKICK_MS
    const bq_pos = postKick ? BALL_Q_POS_POST : BALL_Q_POS_PRE
    const bq_vel = postKick ? BALL_Q_VEL_POST : BALL_Q_VEL_PRE
    ballKF.predict(dt_s, bq_pos, bq_vel)

    // Kalman measurement update (fresh measurements only).
    if (goal_valid) goalKF.update(goal_meas[0], goal_meas[1], GOAL_R_FRESH)
    if (ball_valid) ballKF.update(ball_meas[0], ball_meas[1], BALL_R_FRESH)

    // Use filtered positions once initialized; otherwise fall back to raw measurements.
    ball_now = ball_valid ? ball_meas : [0, 0]
    goal_now = goal_valid ? goal_meas : [0, 0]
    if (ballKF.inited) ball_now = ballKF.pos()
    if (goalKF.inited) goal_now = goalKF.pos()

    // Build the local grid for debugging + condPathClear (ball/goal/kick markers).
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
    }

    // Single Behavior Tree tick per cycle decides walkSpeed/walkTurn/walkMode.
    btRoot()

    if (DEBUG_FLAG && (ball_valid || goal_valid)) {
        if (ball_valid) serial.writeLine(`ball_now x=${ball_now[0]} y=${ball_now[1]}`)
        if (goal_valid) serial.writeLine(`goal_now x=${goal_now[0]} y=${goal_now[1]}`)
        serial.writeLine(`speed=${walkSpeed} turn=${walkTurn} mode=${walkMode}`)
    }

    basic.pause(20)
})

// ---------------------------------------------------------------------------
// 15) Actuator loop: turns walkSpeed/walkTurn/walkMode into actual motion.
// ---------------------------------------------------------------------------
basic.forever(function () {
    if (walkMode == 0) {
        robotPuPro.walk(walkSpeed, walkTurn)
    } else if (walkMode == 1) {
        robotPuPro.walk(-walkSpeed, walkTurn) // back up and turn (alignment)
    } else if (walkMode == 2) {
        lastKickMs = input.runningTime() // tells the Kalman filter to trust ball motion more
        robotPuPro.kick()
    }
    basic.pause(10)
})

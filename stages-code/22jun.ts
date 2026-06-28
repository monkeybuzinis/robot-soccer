/**
 * RobotPU Soccer — FIXED INTEGRATED FINAL PROGRAM
 * ===============================================
 * Strategy kept from the original draft: Opportunistic fallback — approach the
 * ball directly while waiting for the goal, instead of freezing (see
 * btHoldForGoal() below). This is a deliberate divergence from
 * robotpu-soccer-final.ts's "detect both, then plan" strategy -- keep that in
 * mind if you compare behavior against that file.
 *
 * Fixes applied on top of the original draft (these were real MakeCode build
 * errors / functional gaps, confirmed against the actual extension source at
 * pxt-robotpu/main.ts and pxt-robotpu/robotpu.ts):
 *   1. Removed the 6 `declare const ...: any` lines. `basic`, `serial`, `pins`,
 *      `NumberFormat`, `IconNames` are already supplied by the MakeCode
 *      microbit target (see pxt-robotpu/pxt.json's "core"/"radio" deps) --
 *      redeclaring them collided with the real ones ("Cannot redeclare
 *      block-scoped variable").
 *   2. `robotPu` -> `robotPuPro`. Confirmed: pxt-robotpu/main.ts exports
 *      `namespace robotPuPro { ... }`; there is no `robotPu` anywhere in the
 *      compiled extension, so `declare const robotPu: any` only fooled the
 *      type-checker -- the walk()/kick() calls would never actually run.
 *   3. `openSet.splice(bestIdx, 1)` -> `openSet.removeAt(bestIdx)`. MakeCode's
 *      restricted array API doesn't expose `.splice()` (confirmed: the
 *      originally-provided robotpu-A-star.ts uses `.removeAt()` for the exact
 *      same open-set removal step).
 *   4. Added the I2C mux-select + camera service-enable boot sequence. Without
 *      selecting the mux channel and enabling the ball/goal detection services
 *      on the ESP32, `pins.i2cReadBuffer(ESP32_ADDR, ...)` never receives real
 *      detection packets at all -- this was missing entirely from the draft.
 *   5. Wired real head-tracking. `currentYaw`/`currentPitch`/`SEARCH_PATTERN`/
 *      `searchIdx` existed in the draft but were never sent to a servo --
 *      the head never physically moved. Added the same closed-loop
 *      yaw/pitch-byte tracking + head-scan search pattern used in the other
 *      RobotPU files, with the confirmed head-servo range (90° center, pxt-
 *      robotpu's PCB.servoStep() clamps to absolute [0,179]).
 */

// ---------------------------------------------------------------------------
// 1) Interfaces and Globals
// ---------------------------------------------------------------------------
interface Pose2D {
    x: number
    y: number
    theta: number
}

interface Cell {
    i: number
    j: number
}

// --- CONFIGURATION CONSTANTS ---
const MUX_ADDR = 112  // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

// Camera Protocol Targets
const SOCCER_BALL = 0x04
const SOCCER_GOAL = 0x05
const VALID = 1 << 0
const STALE = 1 << 1

const CMD_SERVICE_ENABLE = 8
const SERVICE_WIFI = 1
const SERVICE_IMAGE_CAPTURE = 2
const SERVICE_FACE_DETECTION = 3
const SERVICE_SOCCER_BALL_DETECTION = 4
const SERVICE_SOCCER_GOAL_DETECTION = 5

// Field Constraints & Planning
const GRID_SIZE = 10
const CELL_SIZE = 0.1 // 10 cm per cell
const KICK_DIST_M = 0.15 // Offset behind ball
const ARRIVE_DIST = 0.08 // 8 cm
const LOOKAHEAD_DIST = 0.12
const TURN_GAIN = 2.5
const DEBUG_FLAG = true

// Head servo range: pxt-robotpu's PCB.servoStep() clamps to an ABSOLUTE
// [0,179] degree range, with 90 = looking straight ahead (confirmed against
// the real extension source).
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45
const LOST_TIMEOUT_MS = 6000
const SCAN_WAIT_FRAMES = 25

// Global Shared Actuation Targets
let walkSpeed = 0
let walkTurn = 0
let walkMode = 0 // 0=walk, 1=align, 2=kick

// Head Control State
let yaw = 0
let pitch = 0
let lastBallSeenTime = 0
let search_gain = 1
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
let scanStepIndex = 0
let scanFrameCounter = 0

// ---------------------------------------------------------------------------
// 2) Shared Utilities
// ---------------------------------------------------------------------------
function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

function wrapPi(a: number): number {
    while (a > Math.PI) a -= 2 * Math.PI
    while (a < -Math.PI) a += 2 * Math.PI
    return a
}

function norm2L(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

function i8(v: number): number {
    return v >= 128 ? v - 256 : v
}

function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}

// ---------------------------------------------------------------------------
// 3) Kalman Filter Implementations
// ---------------------------------------------------------------------------
class Kalman2DConstVel {
    public x0: number; public x1: number;
    public P00: number; public P01: number; public P10: number; public P11: number;
    private q: number; private r: number;

    constructor(q: number, r: number) {
        this.q = q; this.r = r;
        this.x0 = 0; this.x1 = 0;
        this.P00 = 10; this.P01 = 0; this.P10 = 0; this.P11 = 10;
    }

    public predict(dt: number): void {
        this.x0 = this.x0 + dt * this.x1;
        this.P00 = this.P00 + dt * (this.P01 + this.P10 + dt * this.P11) + this.q * dt;
        this.P01 = this.P01 + dt * this.P11;
        this.P10 = this.P10 + dt * this.P11;
        this.P11 = this.P11 + this.q * dt;
    }

    public update(z: number): void {
        let s = this.P00 + this.r;
        let k0 = this.P00 / s;
        let k1 = this.P10 / s;
        let y = z - this.x0;
        this.x0 += k0 * y;
        this.x1 += k1 * y;
        this.P00 *= (1 - k0);
        this.P01 *= (1 - k0);
        this.P10 -= k1 * this.P00;
        this.P11 -= k1 * this.P01;
    }

    public pos(): number[] { return [this.x0, this.x1]; }
}

let ballKFX = new Kalman2DConstVel(0.1, 0.5)
let ballKFY = new Kalman2DConstVel(0.1, 0.5)
let goalKFX = new Kalman2DConstVel(0.05, 0.8)
let goalKFY = new Kalman2DConstVel(0.05, 0.8)

// ---------------------------------------------------------------------------
// 4) Viewpoint-to-Viewpoint Steering Core (Layer 4)
// ---------------------------------------------------------------------------
function updateControl(current: Pose2D, target: Pose2D): number[] {
    let kV = 1.5; let kTh = 2.0; let kKn = 1.0;
    let vMax = 1.5; let turnMax = 0.8;

    let dx = target.x - current.x
    let dy = target.y - current.y
    let dist = Math.sqrt(dx * dx + dy * dy)

    if (dist < 0.03) return [0, 0]

    let thetaT = Math.atan2(dy, dx)
    let alpha = wrapPi(thetaT - current.theta)
    let beta = wrapPi(target.theta - thetaT)

    let walkTurnLocal = kTh * alpha + kKn * beta
    walkTurnLocal = clampL(walkTurnLocal, -turnMax, turnMax)

    let walkSpeedLocal = kV * dist * Math.cos(alpha)
    walkSpeedLocal = clampL(walkSpeedLocal, -vMax, vMax)
    if (walkSpeedLocal < 0) walkSpeedLocal = 0

    if (Math.abs(walkTurnLocal) > 0.8 * turnMax) {
        walkSpeedLocal *= 0.5
    }

    return [walkSpeedLocal, walkTurnLocal]
}

// ---------------------------------------------------------------------------
// 5) Grid Mapping Setup
// ---------------------------------------------------------------------------
class LocalGrid {
    public data: Buffer
    constructor() { this.data = pins.createBuffer(GRID_SIZE * GRID_SIZE); this.clear(); }
    public clear() { for (let k = 0; k < this.data.length; k++) this.data[k] = 0; }
    public set(i: number, j: number, val: number) { if (i >= 0 && i < GRID_SIZE && j >= 0 && j < GRID_SIZE) this.data[i * GRID_SIZE + j] = val; }
    public get(i: number, j: number): number { return (i >= 0 && i < GRID_SIZE && j >= 0 && j < GRID_SIZE) ? this.data[i * GRID_SIZE + j] : 1; }
    public index(x: number, y: number): number[] {
        let j = Math.floor((x + (GRID_SIZE * CELL_SIZE) / 2) / CELL_SIZE)
        let i = Math.floor(y / CELL_SIZE)
        return [clampL(i, 0, GRID_SIZE - 1), clampL(j, 0, GRID_SIZE - 1)]
    }
}
let grid = new LocalGrid()

// ---------------------------------------------------------------------------
// 6) A* Algorithm Implementation
// ---------------------------------------------------------------------------
function astarGrid(start: Cell, goal: Cell): Cell[] {
    let rows = GRID_SIZE; let cols = GRID_SIZE
    let openSet: number[] = [start.i * cols + start.j]
    let cameFrom = pins.createBuffer(rows * cols); cameFrom.fill(255)
    let gScore = pins.createBuffer(rows * cols); gScore.fill(100)
    let fScore = pins.createBuffer(rows * cols); fScore.fill(100)

    gScore[start.i * cols + start.j] = 0
    fScore[start.i * cols + start.j] = Math.abs(start.i - goal.i) + Math.abs(start.j - goal.j)

    while (openSet.length > 0) {
        let currentKey = openSet[0]; let bestIdx = 0
        for (let m = 1; m < openSet.length; m++) {
            if (fScore[openSet[m]] < fScore[currentKey]) { currentKey = openSet[m]; bestIdx = m; }
        }

        let curI = Math.idiv(currentKey, cols); let curJ = currentKey - curI * cols
        if (curI == goal.i && curJ == goal.j) {
            let res: Cell[] = []
            let k = currentKey
            while (k != 255) {
                let ci = Math.idiv(k, cols); let cj = k - ci * cols
                res.push({ i: ci, j: cj })
                k = cameFrom[k]
            }
            res.reverse()
            return res
        }

        openSet.removeAt(bestIdx)
        let di = [-1, 1, 0, 0]; let dj = [0, 0, -1, 1]
        for (let n = 0; n < 4; n++) {
            let ni = curI + di[n]; let nj = curJ + dj[n]
            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols && grid.get(ni, nj) != 1) {
                let nKey = ni * cols + nj
                let tentativeG = gScore[currentKey] + 1
                if (tentativeG < gScore[nKey]) {
                    cameFrom[nKey] = currentKey
                    gScore[nKey] = tentativeG
                    fScore[nKey] = tentativeG + Math.abs(ni - goal.i) + Math.abs(nj - goal.j)
                    if (openSet.indexOf(nKey) < 0) openSet.push(nKey)
                }
            }
        }
    }
    return []
}

// ---------------------------------------------------------------------------
// 7) Mathematical Logic Layers (Layer 3)
// ---------------------------------------------------------------------------
function computeKickPoint(ball: number[], goal: number[]): number[] {
    let dx = ball[0] - goal[0]; let dy = ball[1] - goal[1]
    let mag = Math.sqrt(dx * dx + dy * dy)
    if (mag < 0.01) return [ball[0], ball[1] - KICK_DIST_M]
    return [ball[0] + (dx / mag) * KICK_DIST_M, ball[1] + (dy / mag) * KICK_DIST_M]
}

function lookaheadTarget(curPos: { x: number, y: number }, path: { x: number, y: number }[]): { x: number, y: number } {
    if (path.length == 0) return { x: curPos.x, y: curPos.y }
    let bestPt = path[path.length - 1]; let bestDist = 1e9
    for (let k = 0; k < path.length; k++) {
        let d = norm2L(path[k].x - curPos.x, path[k].y - curPos.y)
        if (Math.abs(d - LOOKAHEAD_DIST) < bestDist) { bestDist = Math.abs(d - LOOKAHEAD_DIST); bestPt = path[k]; }
    }
    return bestPt
}

// ---------------------------------------------------------------------------
// 8) Camera service helpers + head-scan search (was missing/disconnected)
// ---------------------------------------------------------------------------
function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

function setSoccerDetection(enabled: boolean) {
    setService(SERVICE_SOCCER_BALL_DETECTION, enabled)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, enabled)
}

let ball_valid = false; let goal_valid = false
let ball_now = [0, 0]; let goal_now = [0, 0]

function searchBall(pattern: { y: number, p: number }[]) {
    yaw *= 0.5
    pitch *= 0.5

    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = pattern[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const liveYaw = robotPuPro.servoTargets()[4]
        const livePitch = robotPuPro.servoTargets()[5]
        const nextYaw = clampL(liveYaw + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(livePitch + targetOffset.p * search_gain, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 1)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 1)
        return
    }

    scanFrameCounter = SCAN_WAIT_FRAMES
    scanStepIndex += 1
    if (scanStepIndex >= SEARCH_PATTERN.length) {
        scanStepIndex = 0
        search_gain = Math.min(4, search_gain * 1.1)
    }
}

// Head-tracks the ball using the packet's yaw/pitch offset bytes (offsets
// 16/17), reading the servo's LIVE current target each cycle (avoids the
// clamp-state drift bug found in earlier drafts of these files).
function trackHead(p: Buffer, count: number, flags: number) {
    const currentTime = input.runningTime()
    if (count > 0) {
        lastBallSeenTime = currentTime
        search_gain = 1.0
        yaw = i8(p[16])
        pitch = i8(p[17])
        const staleScale = (flags & STALE) ? 0.3 : 1.0

        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const liveYaw = robotPuPro.servoTargets()[4]
        const livePitch = robotPuPro.servoTargets()[5]
        const nextYaw = clampL(liveYaw + yaw * staleScale * 0.08, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(livePitch + pitch * staleScale * 0.08, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)
    } else if (currentTime - lastBallSeenTime >= LOST_TIMEOUT_MS) {
        searchBall(SEARCH_PATTERN)
    }
}

// ---------------------------------------------------------------------------
// 9) Behavior Tree-style Routing
// ---------------------------------------------------------------------------
function btRoot() {
    if (condScored()) { btCelebrate(); return; }
    if (ball_valid && goal_valid) { btScore(); return; }
    if (ball_valid) { btHoldForGoal(); return; } // opportunistic: advance on the ball while waiting for the goal
    actionSearchBallBT();
}

function condScored(): boolean {
    return goal_valid && goal_now[1] < 0.25 && Math.abs(goal_now[0]) < 0.15;
}

function btCelebrate() {
    walkSpeed = 0; walkTurn = 0; walkMode = 0
    basic.showIcon(IconNames.Happy)
}

function btScore() {
    let kickPt = computeKickPoint(ball_now, goal_now)
    let distToKick = norm2L(kickPt[0], kickPt[1])
    let headingToGoal = Math.atan2(goal_now[0], goal_now[1])

    if (distToKick < ARRIVE_DIST) {
        if (Math.abs(headingToGoal) < 0.15) { actionKick(); }
        else { actionAlignToGoal(headingToGoal); }
    } else {
        actionApproachKickPoint(kickPt);
    }
}

function btHoldForGoal() {
    // Opportunistic fallback: track and advance directly onto the ball while
    // the goal isn't visible yet, instead of freezing in place.
    let headingBall = Math.atan2(ball_now[0], ball_now[1])
    let distBall = norm2L(ball_now[0], ball_now[1])

    walkTurn = clampL(1.8 * headingBall, -0.6, 0.6)

    if (distBall > 0.22) {
        walkSpeed = 1.0
        walkMode = 0
    } else if (distBall > KICK_DIST_M) {
        walkSpeed = 0.5
        walkMode = 0
    } else {
        walkSpeed = 0
        walkMode = 1
    }
}

function actionApproachKickPoint(kickPt: number[]) {
    let sIdx = grid.index(0, 0); let gIdx = grid.index(kickPt[0], kickPt[1])
    let cellPath = astarGrid({ i: sIdx[0], j: sIdx[1] }, { i: gIdx[0], j: gIdx[1] })

    let wps: { x: number, y: number }[] = []
    for (let k = 0; k < cellPath.length; k++) {
        wps.push({ x: (cellPath[k].j - GRID_SIZE / 2 + 0.5) * CELL_SIZE, y: (cellPath[k].i + 0.5) * CELL_SIZE })
    }

    let tgt = lookaheadTarget({ x: 0, y: 0 }, wps)
    let thT = Math.atan2(goal_now[1] - kickPt[1], goal_now[0] - kickPt[0])

    let ctrl = updateControl({ x: 0, y: 0, theta: 0 }, { x: tgt.x, y: tgt.y, theta: thT })
    walkSpeed = ctrl[0]; walkTurn = ctrl[1]; walkMode = 0
}

function actionAlignToGoal(headingGoal: number) {
    walkTurn = clampL(TURN_GAIN * headingGoal, -0.8, 0.8)
    walkSpeed = 0.3 // Tiny crawl adjustments
    walkMode = 1
}

function actionKick() {
    walkSpeed = 0; walkTurn = 0; walkMode = 2
}

function actionSearchBallBT() {
    walkSpeed = 0; walkTurn = 0; walkMode = 0
    searchBall(SEARCH_PATTERN)
}

// ---------------------------------------------------------------------------
// 10) Boot sequence (was missing -- without this the camera never streams
// detection packets and the robot never responds to remote control)
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

// Handle camera reboot: by default all detection services are off when the
// ESP32 boots, so re-enable them periodically (safe to repeat).
basic.forever(function () {
    setService(SERVICE_IMAGE_CAPTURE, true)
    basic.pause(10)
    setSoccerDetection(true)
    basic.pause(10)
    setService(SERVICE_FACE_DETECTION, false)
    basic.pause(10)
    setService(SERVICE_WIFI, DEBUG_FLAG)
    basic.pause(30000)
})

// ---------------------------------------------------------------------------
// 11) Central Processing & Estimation Loop (20Hz)
// ---------------------------------------------------------------------------
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    ball_valid = false; goal_valid = false

    if (packet.length == SIZE) {
        const flags = packet[3]
        let count = packet[4]
        if (!(flags & VALID)) count = 0

        if (DEBUG_FLAG) {
            serial.writeLine(`type=${packet[0]} flags=${flags} count=${count} x=${i16(packet, 6)} y=${i16(packet, 8)}`)
        }

        const type = packet[0]
        const x_m = i16(packet, 6) / 1000.0
        const y_m = i16(packet, 8) / 1000.0

        if (type === SOCCER_BALL) {
            trackHead(packet, count, flags)
            if (count > 0 && !(flags & STALE)) {
                ballKFX.predict(0.02); ballKFX.update(x_m)
                ballKFY.predict(0.02); ballKFY.update(y_m)
                ball_valid = true
            }
        } else if (type === SOCCER_GOAL) {
            if (count > 0 && !(flags & STALE)) {
                goalKFX.predict(0.02); goalKFX.update(x_m)
                goalKFY.predict(0.02); goalKFY.update(y_m)
                goal_valid = true
            }
        }
    } else {
        serial.writeLine("i2c read error")
    }

    ball_now = [ballKFX.x0, ballKFY.x0]
    goal_now = [goalKFX.x0, goalKFY.x0]

    // Clear and project map objects onto occupancy matrix representation
    grid.clear()
    if (ball_valid) { let b = grid.index(ball_now[0], ball_now[1]); grid.set(b[0], b[1], 2); }
    if (goal_valid) { let g = grid.index(goal_now[0], goal_now[1]); grid.set(g[0], g[1], 3); }

    // Tick the Behavior Tree-style coordinator
    btRoot()

    basic.pause(20)
})

// ---------------------------------------------------------------------------
// 12) Actuator Execution Hardware Driver Loop
// ---------------------------------------------------------------------------
basic.forever(function () {
    if (walkMode == 0 || walkMode == 1) {
        robotPuPro.walk(walkSpeed, walkTurn)
    } else if (walkMode == 2) {
        robotPuPro.walk(0, 0)
        basic.pause(100)
        robotPuPro.kick()
        basic.pause(2500) // Recovery phase hold
    }
    basic.pause(20)
})

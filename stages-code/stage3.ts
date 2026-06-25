/**
 * Stage 3 test (per stages-code/plan.txt): Approach the Ball & Take Up the
 * Kick Position.
 *
 * Objective: with ball + goal both visible, compute a "kick point" (a spot
 * just behind the ball, on the line from the goal through the ball), steer
 * the robot there in a smooth arc, square up to face the goal, and STOP --
 * holding position once aligned. No kicking yet; that is Stage 4.
 *
 * SELF-CONTAINED: paste this single file into MakeCode's main.ts, with the
 * robotPuPro extension attached (needed for locationArray()/servoStep()/
 * walk()/radio control, same as Stage 2).
 *
 * Built from professor-code/robotpu-soccer-mvp.ts + robotpu-localmap.ts +
 * robotpu-kalman-filter.ts + robotpu-viewpoint.ts, but those files have real
 * bugs/mismatches against the actual installed pxt-robotpu extension and
 * against each other. Fixes applied here (confirmed, not stylistic):
 *
 *   1. Namespace/casing: robotPu -> robotPuPro, ServoTargets() ->
 *      servoTargets() (same fix already applied to robotpu-followball.ts).
 *
 *   2. Head servo range: professor-code clamps head yaw/pitch targets to
 *      [-45,45] as an offset from 0. The real robotPuPro.servoStep() takes
 *      an ABSOLUTE [0,179] target with 90 = neutral -- a [-45,45] target
 *      would clamp the head hard against one physical limit. Reused Stage
 *      2's already-hardware-verified HEAD_YAW_CENTER/HEAD_PITCH_CENTER = 90
 *      convention instead.
 *
 *   3. Degrees vs radians: professor-code's camToNow() calls rot(theta, ...)
 *      with theta taken directly from locationArray()[2] and used as-is.
 *      Math.cos/sin expect radians, but robotPuPro.locationArray() returns
 *      theta_deg in DEGREES (confirmed on real hardware logs: e.g.
 *      "theta_deg=106.51..."). Feeding degrees into cos/sin silently produces
 *      a near-arbitrary rotation. Fixed by converting to radians before any
 *      trig call (deg2rad() below).
 *
 *   4. Unit mismatch: professor-code divides camera x_mm/y_mm by 1000 to get
 *      meters for ball_cam2D/goal_cam2D, but then mixes that with
 *      locationArray()'s raw millimeter pose (pose_O) in the SAME camToNow()
 *      add/subtract -- a 1000x scale bug. This file works in millimeters
 *      end-to-end (matching locationArray()'s native unit) and only converts
 *      to radians locally inside trig calls, so there is one consistent unit
 *      everywhere: KICK_BACKOFF_MM, KICK_DIST_MM, the Kalman Q/R constants,
 *      etc. are all millimeter-scale (Q/R variances scaled x1e6 from
 *      professor-code's meter-tuned values, since variance scales with the
 *      square of the unit).
 *
 *   5. Lost-timer bug (same class of bug fixed in Stage 2): professor-code's
 *      trackBall() resets lastBallSeenTime as soon as count>0, even when the
 *      packet is STALE -- so a ball that's physically gone but still being
 *      reported as a stale last-known-position never times out. Fixed: the
 *      lost timer (and the stored cam2D/pose used for camToNow) only resets
 *      on FRESH (non-stale) detections; stale packets still drive a reduced-
 *      gain head-tracking nudge (smooth coast-down) but don't fool the timer.
 *
 *   6. Head-vibration bug (same one just fixed in stage2.ts): search is only
 *      triggered from inside the ball branch once the ball is actually timed
 *      out, never just because the current I2C read happened to return a
 *      goal-type packet. This matters more here than in Stage 2 because both
 *      ball and goal detection run continuously.
 *
 *   7. Kalman filter frame bug (found on hardware: robot walked toward the
 *      ball/goal but kept oscillating and never reached/held the kick pose).
 *      professor-code's Kalman filters smoothed positions AFTER transforming
 *      them into the robot's CURRENT frame ({C_now}, which shifts every
 *      cycle as the robot walks). That conflates the robot's own motion with
 *      the tracked object's velocity -- a perfectly stationary ball looks
 *      like it has nonzero velocity every time the robot itself moves, since
 *      the relative coordinates shift even though the ball didn't. Fixed by
 *      filtering in the fixed ODOM/world frame (camToOdom()), where a
 *      stationary object genuinely has ~0 velocity, then re-projecting the
 *      filtered estimate into {C_now} fresh every cycle with the LIVE pose
 *      (odomToNow()) -- correct even on cycles with no new detection.
 *
 * What this file deliberately does NOT do (out of scope for Stage 3):
 *   - No kicking. walkMode==2 ("at kick point, aligned to goal") just holds
 *     position (walk(0,0)) instead of calling robotPuPro.kick() -- Stage 4
 *     adds exactly that one call.
 *   - No obstacle avoidance / A* (Stage 5). The LocalGrid below is only used
 *     for debug visualization of ball/goal/kick-point cells, matching
 *     robotpu-localmap.ts's original intent, not for path planning yet.
 *
 * How to verify:
 *   1. Show the robot both the ball and the goal. Watch serial for
 *      `ball_now`/`goal_now`/`kickPt` lines updating as you move either one.
 *   2. The robot should walk in a smooth curving approach toward the kick
 *      point (not a straight line + turn-in-place), then rotate to face the
 *      goal, then stop and hold -- log line `AT_KICK_POSE` should print once
 *      and stay steady (not flicker) as long as ball+goal stay visible.
 *   3. Manual override: pushing the remote's walk stick past the deadzone
 *      takes over from the autonomous controller immediately (same as Stage
 *      2), for safety/recovery while testing.
 */

// ---------------------------------------------------------------------------
// I2C protocol / packet layout (from robotpu-i2c-cam.ts) -- ball + goal now
// ---------------------------------------------------------------------------
const MUX_ADDR = 112  // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

const SOCCER_BALL = 0x04
const SOCCER_GOAL = 0x05
const VALID = 1 << 0
const STALE = 1 << 1

function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}

function i8(v: number): number {
    return v >= 128 ? v - 256 : v
}

const CMD_SERVICE_ENABLE = 8
const SERVICE_WIFI = 1
const SERVICE_IMAGE_CAPTURE = 2
const SERVICE_FACE_DETECTION = 3
const SERVICE_SOCCER_BALL_DETECTION = 4
const SERVICE_SOCCER_GOAL_DETECTION = 5

function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

// ---------------------------------------------------------------------------
// Head servo range: ABSOLUTE [0,179], 90 = neutral (Stage 2, hardware-verified)
// ---------------------------------------------------------------------------
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45
// Within this many degrees of HEAD_PITCH_MAX, the camera is pointed nearly
// straight down -- ground-plane distance becomes unreliable there (see the
// planner loop's nearBallByPitch check), so treat a pinned-down pitch as a
// physical "ball is right at the robot's feet" signal.
const NEAR_BALL_PITCH_DEG = HEAD_PITCH_MAX - 10
const HEAD_PITCH_GROUND_BIAS = 15
const HEAD_PITCH_SEARCH_CENTER = HEAD_PITCH_CENTER + HEAD_PITCH_GROUND_BIAS
const HEAD_PITCH_SEARCH_SPAN = 8

const SCAN_WAIT_FRAMES = 25
const DEBUG_FLAG = true

function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

function norm2L(x: number, y: number): number {
    return Math.sqrt(x * x + y * y)
}

function deg2rad(d: number): number {
    return (d * Math.PI) / 180
}

// 2D rotation (theta in RADIANS)
function rot(thetaRad: number, x: number, y: number): number[] {
    const c = Math.cos(thetaRad)
    const s = Math.sin(thetaRad)
    return [c * x - s * y, s * x + c * y]
}

const HEAD_PITCH_SEARCH_MIN = clampL(HEAD_PITCH_SEARCH_CENTER - HEAD_PITCH_SEARCH_SPAN, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
const HEAD_PITCH_SEARCH_MAX = clampL(HEAD_PITCH_SEARCH_CENTER + HEAD_PITCH_SEARCH_SPAN, HEAD_PITCH_MIN, HEAD_PITCH_MAX)

const SEARCH_PATTERN: { y: number, p: number }[] = [
    { y: 35, p: 0 },
    { y: 20, p: 0 },
    { y: 0, p: 0 },
    { y: -20, p: 0 },
    { y: -35, p: 0 },
    { y: -20, p: 5 },
    { y: 0, p: 5 },
    { y: 20, p: 5 },
    { y: 35, p: -5 },
    { y: 0, p: -5 }
]
let scanStepIndex = 0
let scanFrameCounter = 0
let search_gain = 1

function searchBall() {
    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = SEARCH_PATTERN[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const liveYaw = robotPuPro.servoTargets()[4]
        const nextYaw = clampL(liveYaw + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(HEAD_PITCH_SEARCH_CENTER + targetOffset.p, HEAD_PITCH_SEARCH_MIN, HEAD_PITCH_SEARCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 1)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 1)
        if (DEBUG_FLAG) serial.writeLine(`SEARCHING yaw=${nextYaw} pitch=${nextPitch}`)
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
// Local grid (debug visualization only -- not used for planning until Stage 5).
// GRID_N=10 cells at GRID_RES_MM=50mm each -- a 10x10 grid spanning 500mm, not
// 1m cells. Robot is always the center cell (1=robot, 2=ball, 3=goal, 4=kickPt)
// since {C_now} is defined with the robot at (0,0,0) by construction.
// ---------------------------------------------------------------------------
const GRID_N = 10
const GRID_RES_MM = 50
const GRID_HALF_MM = (GRID_N * GRID_RES_MM) / 2

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
    index(x_mm: number, y_mm: number): number[] {
        const j = Math.floor((x_mm + GRID_HALF_MM) / GRID_RES_MM)
        const i = Math.floor(y_mm / GRID_RES_MM)
        return [i, j]
    }
    set(i: number, j: number, v: number) {
        if (this.inBounds(i, j)) this.g[i][j] = v
    }
}
const grid = new LocalGrid()

// ---------------------------------------------------------------------------
// Latency compensation: transform a detection captured at an earlier pose
// into the robot's CURRENT local frame ({C_now}, robot always at (0,0,0)).
// All x/y here are millimeters; theta is converted to radians only for the
// trig calls (see header fix #3/#4).
// ---------------------------------------------------------------------------
// Detection (camera-local, at the pose it was captured) -> odometry/world frame.
// A real-world point's odom-frame coordinates don't change just because the
// robot walks -- this is the frame the Kalman filter below actually runs in.
function camToOdom(cam2D_mm: number[], poseAtDet_mm_deg: number[]): number[] {
    const detThetaRad = deg2rad(poseAtDet_mm_deg[2])
    const detXY = rot(detThetaRad, cam2D_mm[0], cam2D_mm[1])
    return [poseAtDet_mm_deg[0] + detXY[0], poseAtDet_mm_deg[1] + detXY[1]]
}

// Odometry/world frame -> robot's CURRENT local frame ({C_now}, robot at (0,0,0)).
// Called fresh every cycle with the live pose, so it stays correct even on
// cycles with no new detection -- the robot's own motion is always accounted for.
function odomToNow(obj_O: number[], poseNow_mm_deg: number[]): number[] {
    const rel_O = [obj_O[0] - poseNow_mm_deg[0], obj_O[1] - poseNow_mm_deg[1]]
    const nowThetaRad = deg2rad(poseNow_mm_deg[2])
    return rot(-nowThetaRad, rel_O[0], rel_O[1])
}

// ---------------------------------------------------------------------------
// Kalman filter: per-axis constant-velocity [pos, vel], millimeter-scale.
// Q/R values are professor-code's meter-tuned constants x1e6 (variance
// scales with the square of the unit: 1 m^2 = 1e6 mm^2).
// ---------------------------------------------------------------------------
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
        this.P00 = 50000   // 0.05 m^2 x1e6
        this.P01 = 0
        this.P10 = 0
        this.P11 = 1000000 // 1 (m/s)^2 x1e6
    }

    predict(dt_s: number, q_pos: number, q_vel: number) {
        const dt = Math.max(0, dt_s)
        this.x0 = this.x0 + dt * this.x1

        const P00 = this.P00, P01 = this.P01, P10 = this.P10, P11 = this.P11
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

        const P00 = this.P00, P01 = this.P01, P10 = this.P10, P11 = this.P11
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
        if (!this.inited) { this.reset(meas_x, meas_y); return }
        this.kx.update(meas_x, r)
        this.ky.update(meas_y, r)
    }
    pos(): number[] {
        return [this.kx.x0, this.ky.x0]
    }
}

// Goal is stationary: near-constant position.
const GOAL_Q_POS = 1.0     // 1e-6 m^2 x1e6
const GOAL_Q_VEL = 100.0   // 1e-4 m^2/s^2 x1e6
const GOAL_R_FRESH = 800.0 // 8e-4 m^2 x1e6

// Ball: small process noise (it's only rolling/stationary before the kick --
// Stage 4 is where post-kick dynamics would matter).
const BALL_Q_POS = 5.0     // 5e-6 m^2 x1e6
const BALL_Q_VEL = 500.0   // 5e-4 m^2/s^2 x1e6
const BALL_R_FRESH = 1500.0 // 1.5e-3 m^2 x1e6

const ballKF = new Kalman2DConstVel()
const goalKF = new Kalman2DConstVel()
let lastKfMs = -1

// ---------------------------------------------------------------------------
// Kick point / heading helpers (from robotpu-localmap.ts / robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
const KICK_BACKOFF_MM = 50   // 0.05 m
const KICK_DIST_MM = 110     // 0.11 m
const APPROACH_OFFSET_START_MM = 250 // 0.25 m
const TURN_GAIN = -1.2 // flip the sign if the robot turns the wrong direction on hardware

function computeKickPoint(ball_now: number[], goal_now: number[]): number[] {
    const dx = ball_now[0] - goal_now[0]
    const dy = ball_now[1] - goal_now[1]
    const n = Math.max(1e-6, norm2L(dx, dy))
    const ux = dx / n
    const uy = dy / n
    return [ball_now[0] + ux * KICK_BACKOFF_MM, ball_now[1] + uy * KICK_BACKOFF_MM]
}

function desiredHeadingTo(x: number, y: number): number {
    return Math.atan2(x, y)
}

// ---------------------------------------------------------------------------
// Pose-to-pose controller (from robotpu-viewpoint.ts), millimeter-scale.
// Returns [walkSpeed, walkTurn] for robotPuPro.walk(...).
// ---------------------------------------------------------------------------
interface Pose2D { x: number, y: number, theta: number }

function wrapPi(a: number): number {
    while (a > Math.PI) a -= 2 * Math.PI
    while (a <= -Math.PI) a += 2 * Math.PI
    return a
}

function signNonZero(x: number): number {
    return x >= 0 ? 1 : -1
}

function updateControl(current: Pose2D, target: Pose2D, offsetStartDist_mm: number, stopDist_mm: number): number[] {
    const vMax = 2.5
    const turnMax = 0.8
    const kTurn = -2.0 // if robot turns the wrong direction, flip the subtraction order below

    const leadMin_mm = 50
    const leadMax_mm = 180
    const lateralOffsetMax_mm = 100

    const dx = target.x - current.x
    const dy = target.y - current.y
    const dist = norm2L(dx, dy)

    if (dist < stopDist_mm) return [0, 0]

    const tx = Math.sin(target.theta)
    const ty = Math.cos(target.theta)
    const nx = -ty
    const ny = tx

    const vx = current.x - target.x
    const vy = current.y - target.y
    const cross = tx * vy - ty * vx
    const side = -signNonZero(cross)

    const offsetGain = clampL(1.0 - dist / offsetStartDist_mm, 0.0, 1.0)
    const lead = leadMin_mm + (leadMax_mm - leadMin_mm) * offsetGain
    const lateral = (lateralOffsetMax_mm * offsetGain) * side

    const xV = target.x + lead * tx + lateral * nx
    const yV = target.y + lead * ty + lateral * ny

    const headingToV = Math.atan2(xV - current.x, yV - current.y)
    const eHeading = wrapPi(headingToV - current.theta)
    const walkTurn = clampL(kTurn * eHeading, -turnMax, turnMax)

    let walkSpeed = vMax
    if (Math.abs(walkTurn) > 0.9 * turnMax) walkSpeed *= 0.6

    return [walkSpeed, walkTurn]
}

// ---------------------------------------------------------------------------
// Boot sequence: remote control + camera mux/service enable
// ---------------------------------------------------------------------------
robotPuPro.setChannel(166)
robotPuPro.setServoTrim(-5, 0, -5, 0, -8, 0)
robotPuPro.setEyeBrightness(0) // the extension blinks the eye LEDs by default in the background; not needed here

const REMOTE_FWD_SPEED = 3
const REMOTE_BWD_SPEED = -2
const REMOTE_STICK_DEADZONE = 0.2
let remoteWalkSpeed = 0
let remoteWalkTurn = 0

radio.onReceivedString(function (receivedString: string) {
    robotPuPro.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name: string, value: number) {
    if (name == "#puspeed") {
        if (value > REMOTE_STICK_DEADZONE) remoteWalkSpeed = value * REMOTE_FWD_SPEED
        else if (value < -REMOTE_STICK_DEADZONE) remoteWalkSpeed = -value * REMOTE_BWD_SPEED
        else remoteWalkSpeed = 0
    } else if (name == "#puturn") {
        remoteWalkTurn = (remoteWalkTurn * 4 + value) * 0.2
    } else {
        robotPuPro.runKeyValueCommand(name, value)
    }
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

basic.showString("3")
pins.i2cWriteNumber(MUX_ADDR, 0x0F, NumberFormat.Int8LE, false)
basic.pause(2000)

basic.forever(function () {
    setService(SERVICE_IMAGE_CAPTURE, true)
    basic.pause(10)
    setService(SERVICE_FACE_DETECTION, false)
    basic.pause(10)
    setService(SERVICE_SOCCER_BALL_DETECTION, true)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, true)
    basic.pause(30000)
})

// ---------------------------------------------------------------------------
// Detection state stored at image time -- cam2D in millimeters (ground-plane,
// x=left/right, y=forward), pose in [x_mm, y_mm, theta_deg] from
// robotPuPro.locationArray() at the moment of that FRESH (non-stale) reading.
// ---------------------------------------------------------------------------
// Shorter than Stage 2's ball-only timeout: once both ball+goal are valid the
// body is actively walking off the Kalman-predicted ball_now, so a long grace
// window means several hundred ms of walking on a stale/extrapolated position
// after the camera has actually lost the ball -- this is the "walked past the
// ball without seeing it" behavior observed on hardware.
const BALL_LOST_TIMEOUT_MS = 500
const GOAL_LOST_TIMEOUT_MS = 3000

let ball_cam2D_mm: number[] = [0, 0]
let ball_pose_mm_deg: number[] = [0, 0, 0]
let ball_valid = false
let lastBallSeenMs = -999999
let lastYawByte = 0
let lastPitchByte = 0

let goal_cam2D_mm: number[] = [0, 0]
let goal_pose_mm_deg: number[] = [0, 0, 0]
let goal_valid = false
let lastGoalSeenMs = -999999

// Sanity bound on raw camera distance (mm from the camera, not the robot-
// frame projection). Real ball/goal sightings on a small field never get
// anywhere near this; values like x_mm=1701 y_mm=4580 in device logs were
// the camera mistaking ceiling lights/walls for the goal while pitched up.
const MAX_DETECT_DIST_MM = 2500

function trackPacket(p: Buffer) {
    const type = p[0]
    const flags = p[3]
    const count = p[4]
    const isValid = (flags & VALID) != 0 && count > 0
    const isStale = (flags & STALE) != 0
    const nowMs = input.runningTime()

    if (type == SOCCER_BALL) {
        const x_mm = i16(p, 6)
        const y_mm = i16(p, 8)
        const plausible = norm2L(x_mm, y_mm) <= MAX_DETECT_DIST_MM

        if (isValid && !isStale && plausible) {
            // Head-tracking ported from robotpu-followball.ts (confirmed
            // working on real hardware). The earlier version here (0.08
            // gain, staleScale, a "corrected" sign flip) still lost the ball
            // while walking on hardware -- this is the version that's
            // actually been verified, so it replaces that one rather than
            // patching it further.
            lastBallSeenMs = nowMs
            search_gain = 1.0
            lastYawByte = i8(p[16])
            lastPitchByte = i8(p[17])

            robotPuPro.setModeVar(robotPuPro.Mode.API)
            const nextYaw = clampL(robotPuPro.servoTargets()[4] + lastYawByte * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(robotPuPro.servoTargets()[5] + lastPitchByte * 0.2, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)

            const pose = robotPuPro.locationArray()
            ball_cam2D_mm = [x_mm, y_mm]
            ball_pose_mm_deg = [pose[0], pose[1], pose[2]]
            ball_valid = true

            if (DEBUG_FLAG) serial.writeLine(`BALL_TRACK yaw=${nextYaw} pitch=${nextPitch}`)
        } else if (nowMs - lastBallSeenMs < BALL_LOST_TIMEOUT_MS) {
            // Brief dropout: decay follow-through (same as robotpu-followball.ts)
            // instead of immediately searching -- avoids head-snapping on a
            // single missed/stale frame.
            lastYawByte *= 0.7
            lastPitchByte *= 0.7
            const nextYaw = clampL(robotPuPro.servoTargets()[4] + lastYawByte * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(robotPuPro.servoTargets()[5] + lastPitchByte * 0.2, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 5)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 5)
        } else {
            ball_valid = false
            searchBall()
        }
    } else if (type == SOCCER_GOAL) {
        const x_mm = i16(p, 6)
        const y_mm = i16(p, 8)
        const plausible = norm2L(x_mm, y_mm) <= MAX_DETECT_DIST_MM

        if (isValid && !isStale && plausible) {
            lastGoalSeenMs = nowMs
            const pose = robotPuPro.locationArray()
            goal_cam2D_mm = [x_mm, y_mm]
            goal_pose_mm_deg = [pose[0], pose[1], pose[2]]
            goal_valid = true
            if (DEBUG_FLAG) serial.writeLine(`GOAL_SEEN x_mm=${x_mm} y_mm=${y_mm}`)
        } else if (nowMs - lastGoalSeenMs >= GOAL_LOST_TIMEOUT_MS) {
            goal_valid = false
        }
        if (DEBUG_FLAG && isValid && !isStale && !plausible) {
            serial.writeLine(`GOAL_REJECTED x_mm=${x_mm} y_mm=${y_mm} (implausible -- likely ceiling/wall noise)`)
        }
    }
}

// ---------------------------------------------------------------------------
// Planner: map detections into {C_now}, filter, compute the kick point, and
// drive there. walkMode 2 ("aligned, at kick point") just holds in this
// stage -- Stage 4 is the one-line addition of robotPuPro.kick() there.
// ---------------------------------------------------------------------------
let walkSpeed = 0
let walkTurn = 0
let walkMode = 0

basic.forever(function () {
    const packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackPacket(packet)
    } else if (DEBUG_FLAG) {
        serial.writeLine(`I2C_ERR len=${packet.length}`)
    }

    const nowMs = input.runningTime()
    const poseNow = robotPuPro.locationArray() // [x_mm, y_mm, theta_deg]

    if (ball_valid && nowMs - lastBallSeenMs >= BALL_LOST_TIMEOUT_MS) ball_valid = false
    if (goal_valid && nowMs - lastGoalSeenMs >= GOAL_LOST_TIMEOUT_MS) goal_valid = false

    const haveBallMeas = ball_valid
    const haveGoalMeas = goal_valid

    // Filter in the ODOM/world frame, not the robot's current frame: a
    // stationary ball has ~0 velocity in odom, but would falsely appear to
    // have nonzero velocity in {C_now} every time the robot itself moves,
    // since the relative coordinates shift even though the ball didn't.
    // Filtering post-transform conflated robot motion with object motion,
    // so the kick-point estimate kept drifting/oscillating and the robot
    // never converged close enough to ever leave walkMode 0 on hardware.
    if (lastKfMs < 0) lastKfMs = nowMs
    const dt_s = Math.min(0.2, Math.max(0, (nowMs - lastKfMs) / 1000))
    lastKfMs = nowMs

    goalKF.predict(dt_s, GOAL_Q_POS, GOAL_Q_VEL)
    ballKF.predict(dt_s, BALL_Q_POS, BALL_Q_VEL)
    if (haveGoalMeas) {
        const goal_meas_O = camToOdom(goal_cam2D_mm, goal_pose_mm_deg)
        goalKF.update(goal_meas_O[0], goal_meas_O[1], GOAL_R_FRESH)
    }
    if (haveBallMeas) {
        const ball_meas_O = camToOdom(ball_cam2D_mm, ball_pose_mm_deg)
        ballKF.update(ball_meas_O[0], ball_meas_O[1], BALL_R_FRESH)
    }

    // Re-project the filtered odom-frame estimate into {C_now} fresh every
    // cycle using the LIVE pose -- this stays correct even on cycles with no
    // new detection, since it's the robot's own motion (not a new sample)
    // driving the change.
    let ball_now = [0, 0]
    let goal_now = [0, 0]
    if (ballKF.inited) ball_now = odomToNow(ballKF.pos(), poseNow)
    if (goalKF.inited) goal_now = odomToNow(goalKF.pos(), poseNow)

    grid.clear()
    // Robot is always the grid's center cell -- {C_now} is defined with the
    // robot at (0,0,0) by construction, so this never needs odom/pose math.
    const idxR = grid.index(0, 0)
    grid.set(idxR[0], idxR[1], 1)
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

        // Ground-plane distance from a single downward-pitched camera is only
        // reliable far from the robot -- near the camera's blind spot (head
        // pitched almost straight down) small pitch errors blow up into large
        // distance errors, so the geometric distKick estimate can stay stuck
        // well above KICK_DIST_MM even as the ball is right at the robot's
        // feet (confirmed on hardware: robot walked into/past the ball
        // without ever satisfying distKick <= KICK_DIST_MM). Use head pitch
        // itself as a physical "ball is right here" cue once it's pinned
        // near its downward limit, instead of trusting distance alone.
        const headPitchNow = robotPuPro.servoTargets()[5]
        const nearBallByPitch = ball_valid && headPitchNow >= NEAR_BALL_PITCH_DEG

        if (distKick > KICK_DIST_MM && !nearBallByPitch) {
            const ctrl = updateControl(
                { x: 0, y: 0, theta: 0 },
                { x: kickPt[0], y: kickPt[1], theta: thetaKick },
                APPROACH_OFFSET_START_MM,
                KICK_DIST_MM
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

        if (DEBUG_FLAG) {
            serial.writeLine(`ball_now x=${ball_now[0]} y=${ball_now[1]}`)
            serial.writeLine(`goal_now x=${goal_now[0]} y=${goal_now[1]}`)
            serial.writeLine(`kickPt x=${kickPt[0]} y=${kickPt[1]} dist=${distKick} mode=${walkMode}`)
            if (walkMode == 2) serial.writeLine("AT_KICK_POSE")
        }
    } else {
        // Ball-only (goal not visible yet) or neither: don't walk. The body
        // only ever moves once both ball+goal are valid and a real kick
        // point can be computed -- head tracking/searching above still runs
        // independently, so the ball stays centered while the body holds.
        walkSpeed = 0
        walkTurn = 0
        walkMode = 0
    }

    basic.pause(20)
})

// ---------------------------------------------------------------------------
// Actuator loop. Remote stick (if pushed) always overrides the autonomous
// controller, same safety/recovery behavior as Stage 2.
// ---------------------------------------------------------------------------
basic.forever(function () {
    if (remoteWalkSpeed != 0) {
        robotPuPro.walk(remoteWalkSpeed, remoteWalkTurn)
    } else if (walkMode == 0) {
        robotPuPro.walk(walkSpeed, walkTurn)
    } else if (walkMode == 1) {
        robotPuPro.walk(-walkSpeed, walkTurn) // back up and turn to align with the goal
    } else {
        // walkMode == 2: aligned at the kick point. Stage 4 replaces this
        // line with a repeated robotPuPro.kick() call -- nothing else here
        // needs to change.
        robotPuPro.walk(0, 0)
    }
    basic.pause(20)
})

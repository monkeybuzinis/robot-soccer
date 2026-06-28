/**
 * Stage 4 test (per stages-code/plan.txt): Approach the Ball, Take Up the
 * Kick Position, and Kick.
 *
 * Objective: with ball + goal both visible, compute a "kick point" (a spot
 * just behind the ball, on the line from the goal through the ball), steer
 * the robot there in a smooth arc, square up to face the goal, and kick.
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
 * Stage 4 changes from Stage 3:
 *   - Fixes Stage 3's early "at kick pose" bug. Hardware logs showed
 *     kickPt dist around 280mm, then mode=2/AT_KICK_POSE. That is not
 *     odometry drift: the filtered ball/goal/kickPt values were stable.
 *     The bug was the code's too-large arrival threshold plus a pitch-only
 *     shortcut that could declare success before the robot reached the
 *     kick point.
 *   - walkMode==2 now calls robotPuPro.kick().
 *   - No obstacle avoidance / A* (Stage 5). The LocalGrid below is only used
 *     for debug visualization of ball/goal/kick-point cells, matching
 *     robotpu-localmap.ts's original intent, not for path planning yet.
 *
 * How to verify:
 *   1. Show the robot both the ball and the goal. Watch serial for
 *      `ball_now`/`goal_now`/`kickPt` lines updating as you move either one.
 *   2. The robot should walk in a smooth curving approach toward the kick
 *      point (not a straight line + turn-in-place), then rotate to face the
 *      goal, then kick -- log line `AT_KICK_POSE` should only print once
 *      the kick point is genuinely close, not while distKick is still
 *      around 200-300mm.
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

const VISION_BALL_MODE = 0
const VISION_GOAL_MODE = 1
const VISION_SLOT_MS = 140
let visionSlot = 0
let visionPreferGoal = false
let visionAcquireGoal = false

function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

function selectVisionTarget(target: number) {
    // The camera returns one event packet per I2C read, so enabling both
    // detectors can starve the lower-confidence object. Force alternating
    // service windows so ball and goal each get fresh packet opportunities.
    setService(SERVICE_SOCCER_BALL_DETECTION, target == VISION_BALL_MODE)
    setService(SERVICE_SOCCER_GOAL_DETECTION, target == VISION_GOAL_MODE)
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
// REVERSED from earlier this session. The previous "increasing pitch = up"
// conclusion was based on ambiguous evidence (head pinned at HEAD_PITCH_MAX
// while losing sight of ball/goal -- equally explainable as pinned DOWN at
// the floor as pinned UP at the ceiling). New hardware log (search sweeping
// pitch=62.5-72.5, entirely BELOW HEAD_PITCH_CENTER=90) still visibly faced
// above level -- which only makes sense if LOWER pitch is up and HIGHER
// pitch is down. Corrected convention: HEAD_PITCH_MIN (45) = up extreme,
// HEAD_PITCH_CENTER (90) = level, HEAD_PITCH_MAX (135) = down extreme
// (toward the robot's own feet).
//
// Within this many degrees of HEAD_PITCH_MAX (down), the camera is pointed
// at the robot's own feet -- ground-plane distance becomes unreliable there
// (see the planner loop's nearBallByPitch check), so treat a pinned-down
// pitch as a physical "ball is right at the robot's feet" signal.
const NEAR_BALL_PITCH_DEG = HEAD_PITCH_MAX - 10
// Ball and goal are never above camera level when the robot's base is
// parallel to the ground, so there is never a legitimate reason for this
// robot to pitch its head above level (90/HEAD_PITCH_CENTER) at all -- not
// while tracking, not while searching. Under the corrected convention above,
// "above level" means LOWER than HEAD_PITCH_CENTER, so the operating range
// is a FLOOR at level, not a ceiling. HEAD_PITCH_OPERATING_MIN replaces
// HEAD_PITCH_MIN as the pitch clamp floor everywhere in this file (tracking,
// dropout-decay, search, and the pin-detection threshold below), so pitch
// can structurally never go above level in the first place.
const HEAD_PITCH_OPERATING_MIN = HEAD_PITCH_CENTER
// Search sweeps the full level-and-below band: from level (90) down to the
// physical down limit (135), not a narrow slice of it.
const HEAD_PITCH_SEARCH_MIN = HEAD_PITCH_OPERATING_MIN
const HEAD_PITCH_SEARCH_MAX = HEAD_PITCH_MAX
const HEAD_PITCH_SEARCH_CENTER = (HEAD_PITCH_SEARCH_MIN + HEAD_PITCH_SEARCH_MAX) / 2

const SCAN_WAIT_FRAMES = 12 // was 25 -- halved now that servoStep speed below is also faster
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
let searchMode = 0

function setSearchMode(mode: number) {
    if (searchMode == mode) return
    searchMode = mode
    scanStepIndex = 0
    scanFrameCounter = 0
    search_gain = 1
}

function searchBall() {
    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = SEARCH_PATTERN[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        // Absolute target from HEAD_YAW_CENTER, same as pitch below -- not
        // built on the live yaw. Building it on live yaw re-added the same
        // offset on top of an already-shifted position every held frame
        // (scanFrameCounter keeps this step active for SCAN_WAIT_FRAMES
        // cycles), so it ratcheted into the clamp within 1-2 frames instead
        // of moving to a fixed offset from center.
        const nextYaw = clampL(HEAD_YAW_CENTER + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        // Back to addition (convention reversed again -- see HEAD_PITCH_MIN/
        // MAX comment above): HEAD_PITCH_SEARCH_CENTER now sits in the
        // level-to-down half (112.5, between 90 and 135), and higher pitch
        // is DOWN, so p:+5 pushing toward center+5 means further down, and
        // p:-5 toward center-5 means back up toward level -- the intuitive
        // direction the pattern's signs were originally written for.
        const nextPitch = clampL(HEAD_PITCH_SEARCH_CENTER + targetOffset.p, HEAD_PITCH_SEARCH_MIN, HEAD_PITCH_SEARCH_MAX)
        // Speed was 1 (the slowest setting) -- much slower than the live
        // tracking calls (5/8) elsewhere in this file, which is why search
        // visibly crept one degree at a time on hardware. Matched to the
        // fresh-track speed so the sweep is actually brisk.
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)
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

function searchGoal() {
    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = SEARCH_PATTERN[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const nextYaw = clampL(HEAD_YAW_CENTER + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        // Goal is farther and taller than the ball, so scan near level and
        // slightly downward instead of pinning the head at the floor.
        const nextPitch = clampL(HEAD_PITCH_CENTER + targetOffset.p, HEAD_PITCH_CENTER, HEAD_PITCH_CENTER + 25)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)
        if (DEBUG_FLAG) serial.writeLine(`SEARCH_GOAL yaw=${nextYaw} pitch=${nextPitch}`)
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
//
// IMPORTANT: cam2D_mm is in the CAMERA's own frame (x_mm/y_mm relative to
// wherever the head is currently pointed), NOT the robot body's frame. The
// project doc's MVP shortcut of treating (x_mm, y_mm) directly as a body-
// frame measurement is only valid "when head yaw/pitch stays near zero
// during walking" -- but this script actively swings the head to track/
// search (yaw 45-135 deg), so that assumption doesn't hold here. Missing
// this correction meant every ball_now/goal_now computed while the head was
// off-center was silently wrong, worse the further off-center it was --
// almost certainly the source of the wild ball_now/goal_now swings seen on
// hardware (positions jumping to 2000+mm on a ~1m field). Fixed by rotating
// the camera-frame offset by the head's yaw deviation from center BEFORE
// applying the body's own world rotation.
//
// Rotation sign is unverified on hardware (same caveat as TURN_GAIN/kTurn
// elsewhere in this file) -- if correcting this makes positions swing MORE
// with head deflection instead of less, flip headYawOffsetRad's sign.
function camToOdom(cam2D_mm: number[], headYawDeg: number, poseAtDet_mm_deg: number[]): number[] {
    const headYawOffsetRad = deg2rad(headYawDeg - HEAD_YAW_CENTER)
    const bodyFrameXY = rot(headYawOffsetRad, cam2D_mm[0], cam2D_mm[1])
    const detThetaRad = deg2rad(poseAtDet_mm_deg[2])
    const detXY = rot(detThetaRad, bodyFrameXY[0], bodyFrameXY[1])
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
const KICK_BACKOFF_MM = 20   // 0.02 m, matched to the current ~2cm ball
// Stage 3 used 320mm after an earlier calibration run, but the latest device
// log shows that this declares AT_KICK_POSE while kickPt is still ~280mm
// away. Hardware testing then showed even 110mm can leave the robot paused
// more than 10cm behind the ball, so keep the stop gate very tight and let
// the retry logic recover if the ball slips.
const KICK_DIST_MM = 10
// If the head is pinned down and the kick point is already fairly close, let
// pitch act as a backup close-range cue. It must not override a large
// distKick by itself; that was the early-stop bug.
const KICK_PITCH_ASSIST_DIST_MM = 15
const APPROACH_OFFSET_START_MM = 450
const TURN_GAIN = -1.2 // flip the sign if the robot turns the wrong direction on hardware
const ALIGN_HEADING_TOL = 0.25
// There is no "ball crossed goal line" event in the I2C packet. Stop once
// the filtered ball and goal positions are close enough to count as scored.
const SCORE_DIST_MM = 120
// kick() is a motion macro, so call it for a short window, then re-check
// vision. If the ball slipped away or was missed, the planner falls back to
// re-approach/re-align instead of kicking forever.
const KICK_ACTION_MS = 1200
// Hardware convention check from the 2026-06-28 log: with a positive approach
// speed and a kick point in front of the robot, both ball_now and goal_now
// moved farther away until the objects were behind the robot. Keep planner
// coordinates camera-forward-positive, but invert the body gait command here.
const BODY_WALK_SIGN = -1

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
    const kTurn = 2.0 // hardware turn sign: positive eHeading should turn toward +x target

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
// setEyeBrightness(0) alone doesn't actually turn the eyes off: the extension's
// RobotPu constructor calls pcb.eyesCtl(1) once at boot (a DIGITAL pin write,
// full brightness, bypassing the brightness scalar entirely), and the only
// thing that would ever turn it back off is the background blink() animation
// -- which never runs here because this script forces gst=Mode.API on every
// cycle (blink() only fires while gst is in [0,5]). Directly overriding the
// analog pins once at boot is the only call that actually sticks.
robotPuPro.leftEyeBright(0)
robotPuPro.rightEyeBright(0)

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

    const goalEvery = visionPreferGoal ? 2 : 4
    const target = visionAcquireGoal ? VISION_GOAL_MODE : ((visionSlot % goalEvery == goalEvery - 1) ? VISION_GOAL_MODE : VISION_BALL_MODE)
    selectVisionTarget(target)

    if (target == VISION_GOAL_MODE && visionPreferGoal && !visionAcquireGoal) {
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, HEAD_YAW_CENTER, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, HEAD_PITCH_CENTER, 8)
        if (DEBUG_FLAG) serial.writeLine("VISION_GOAL_SAMPLE")
    } else if (DEBUG_FLAG && target == VISION_GOAL_MODE) {
        serial.writeLine("VISION_GOAL_SAMPLE")
    }

    visionSlot += 1
    basic.pause(VISION_SLOT_MS)
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
// Must be longer than the ball/goal vision service rotation. With a 140ms
// slot and periodic goal-only windows, 500ms can falsely mark the ball lost
// after only a couple of walking steps even when the ball is still present.
const BALL_LOST_TIMEOUT_MS = 1200
const GOAL_LOST_TIMEOUT_MS = 3000

// Head yaw AT DETECTION TIME, snapshotted alongside cam2D/pose. camToOdom()
// needs this to correct for the camera not pointing straight ahead -- see
// the comment on camToOdom() itself for why.
let ball_cam2D_mm: number[] = [0, 0]
let ball_pose_mm_deg: number[] = [0, 0, 0]
let ball_head_yaw_deg = HEAD_YAW_CENTER
let ball_valid = false
let lastBallSeenMs = -999999
let lastYawByte = 0
let lastPitchByte = 0

let goal_cam2D_mm: number[] = [0, 0]
let goal_pose_mm_deg: number[] = [0, 0, 0]
let goal_head_yaw_deg = HEAD_YAW_CENTER
let goal_valid = false
let lastGoalSeenMs = -999999
let freshBallThisCycle = false
let freshGoalThisCycle = false
let kickPlanReady = false

// Safety net against the head-runaway-up bug observed on hardware (head
// drifts up and freezes while "tracking" the ball, losing both ball and
// goal): pitch can no longer physically go below HEAD_PITCH_OPERATING_MIN
// (level) now, but a genuinely tracked ball still never needs the head
// pinned at that floor for long -- that's camera hunting noise at the
// horizon, not a real detection. After this many consecutive fresh-
// detection cycles pinned at the floor, force the ball lost and let
// search() recenter the head, instead of continuing to trust/drive on it.
// Pinned at HEAD_PITCH_MAX (down) is NOT covered here -- that's the
// legitimate "ball right at the robot's feet" case (see nearBallByPitch
// below).
const PITCH_PIN_UP_MARGIN_DEG = 3
const PITCH_PIN_UP_CYCLE_LIMIT = 15
let pitchPinnedUpCycles = 0

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
            // Yaw the head was AT when this packet's image was captured
            // (approximately, modulo latency) -- snapshotted before this
            // cycle's nudge, for camToOdom()'s head-yaw correction.
            const yawAtDetection = robotPuPro.servoTargets()[4]
            const nextYaw = clampL(yawAtDetection + lastYawByte * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(robotPuPro.servoTargets()[5] + lastPitchByte * 0.2, HEAD_PITCH_OPERATING_MIN, HEAD_PITCH_MAX)

            if (!kickActive && !kickPlanReady) {
                robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
                robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)

                if (nextPitch <= HEAD_PITCH_OPERATING_MIN + PITCH_PIN_UP_MARGIN_DEG) {
                    pitchPinnedUpCycles += 1
                } else {
                    pitchPinnedUpCycles = 0
                }
            } else {
                pitchPinnedUpCycles = 0
            }

            if (pitchPinnedUpCycles >= PITCH_PIN_UP_CYCLE_LIMIT) {
                // Pinned looking up for too long while supposedly tracking --
                // this is the runaway-up bug, not a real detection. Force
                // lost. Before a kick plan exists, the planner loop will
                // search; after a kick plan exists, odometry continues the
                // approach and fresh packets are only optional corrections.
                pitchPinnedUpCycles = 0
                ball_valid = false
                if (DEBUG_FLAG) serial.writeLine("BALL_PITCH_PINNED_UP -- forcing lost")
                return
            }

            const pose = robotPuPro.locationArray()
            ball_cam2D_mm = [x_mm, y_mm]
            ball_pose_mm_deg = [pose[0], pose[1], pose[2]]
            ball_head_yaw_deg = yawAtDetection
            ball_valid = true
            freshBallThisCycle = true

            if (DEBUG_FLAG) serial.writeLine(`BALL_TRACK yaw=${nextYaw} pitch=${nextPitch}`)
        } else if (nowMs - lastBallSeenMs < BALL_LOST_TIMEOUT_MS) {
            // Brief dropout: decay follow-through (same as robotpu-followball.ts)
            // instead of immediately searching -- avoids head-snapping on a
            // single missed/stale frame.
            lastYawByte *= 0.7
            lastPitchByte *= 0.7
            const nextYaw = clampL(robotPuPro.servoTargets()[4] + lastYawByte * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(robotPuPro.servoTargets()[5] + lastPitchByte * 0.2, HEAD_PITCH_OPERATING_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 5)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 5)
        } else {
            // Lost beyond the decay window. Don't call searchBall() here --
            // see the planner loop below for why it's centralized there now.
            if (!kickPlanReady) ball_valid = false
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
            // The goal branch never moves the head itself, but cam2D_mm is
            // still relative to wherever the head currently is, driven by
            // ball tracking/search -- snapshot the current yaw, same as the
            // ball branch does.
            goal_head_yaw_deg = robotPuPro.servoTargets()[4]
            goal_valid = true
            freshGoalThisCycle = true
            if (DEBUG_FLAG) serial.writeLine(`GOAL_SEEN x_mm=${x_mm} y_mm=${y_mm}`)
        } else if (!kickPlanReady && nowMs - lastGoalSeenMs >= GOAL_LOST_TIMEOUT_MS) {
            goal_valid = false
        }
        if (DEBUG_FLAG && isValid && !isStale && !plausible) {
            serial.writeLine(`GOAL_REJECTED x_mm=${x_mm} y_mm=${y_mm} (implausible -- likely ceiling/wall noise)`)
        }
    }
}

// ---------------------------------------------------------------------------
// Planner: map detections into {C_now}, filter, compute the kick point, drive
// there, then set walkMode 2 once aligned so the actuator loop kicks.
// ---------------------------------------------------------------------------
let walkSpeed = 0
let walkTurn = 0
let walkMode = 0
let kickActive = false
let kickStartMs = 0
let kickStarted = false
let scored = false

function isScoredByVision(ball_now: number[], goal_now: number[]): boolean {
    return norm2L(ball_now[0] - goal_now[0], ball_now[1] - goal_now[1]) <= SCORE_DIST_MM
}

basic.forever(function () {
    freshBallThisCycle = false
    freshGoalThisCycle = false

    const packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackPacket(packet)
    } else if (DEBUG_FLAG) {
        serial.writeLine(`I2C_ERR len=${packet.length}`)
    }

    const nowMs = input.runningTime()
    const poseNow = robotPuPro.locationArray() // [x_mm, y_mm, theta_deg]

    if (!kickPlanReady && !visionAcquireGoal && ball_valid && nowMs - lastBallSeenMs >= BALL_LOST_TIMEOUT_MS) ball_valid = false
    if (!kickPlanReady && goal_valid && nowMs - lastGoalSeenMs >= GOAL_LOST_TIMEOUT_MS) goal_valid = false

    // Centralized here instead of inside trackPacket()'s SOCCER_BALL branch:
    // the camera emits one event type per I2C packet, and Stage 4 actively
    // alternates ball/goal service windows. Search is only for acquisition;
    // once ball+goal have initialized an odometry-frame plan, missing vision
    // packets should not stop the approach.
    if (!kickPlanReady) {
        if (!ball_valid) {
            visionAcquireGoal = false
            visionPreferGoal = false
            setSearchMode(VISION_BALL_MODE)
            searchBall()
        } else if (!goal_valid) {
            visionAcquireGoal = true
            visionPreferGoal = true
            setSearchMode(VISION_GOAL_MODE)
            searchGoal()
        } else {
            visionAcquireGoal = false
            visionPreferGoal = false
        }
    }

    // Stage 4 treats the ball/goal as fixed once both have initialized the
    // odometry-frame plan. Later packets are diagnostics; they should not
    // move the target while the robot is already walking to it.
    const haveBallMeas = freshBallThisCycle && !kickPlanReady
    const haveGoalMeas = freshGoalThisCycle && !kickPlanReady

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
        const goal_meas_O = camToOdom(goal_cam2D_mm, goal_head_yaw_deg, goal_pose_mm_deg)
        goalKF.update(goal_meas_O[0], goal_meas_O[1], GOAL_R_FRESH)
        if (DEBUG_FLAG) serial.writeLine(`goal_O x=${goalKF.pos()[0]} y=${goalKF.pos()[1]}`)
    }
    if (haveBallMeas) {
        const ball_meas_O = camToOdom(ball_cam2D_mm, ball_head_yaw_deg, ball_pose_mm_deg)
        ballKF.update(ball_meas_O[0], ball_meas_O[1], BALL_R_FRESH)
        if (DEBUG_FLAG) serial.writeLine(`ball_O x=${ballKF.pos()[0]} y=${ballKF.pos()[1]}`)
    }
    if (ballKF.inited && goalKF.inited) kickPlanReady = true

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
    if (ballKF.inited) {
        const idxB = grid.index(ball_now[0], ball_now[1])
        grid.set(idxB[0], idxB[1], 2)
    }
    if (goalKF.inited) {
        const idxG = grid.index(goal_now[0], goal_now[1])
        grid.set(idxG[0], idxG[1], 3)
    }

    if (kickPlanReady) {
        const kickPt = computeKickPoint(ball_now, goal_now)
        const idxK = grid.index(kickPt[0], kickPt[1])
        grid.set(idxK[0], idxK[1], 4)

        const distKick = norm2L(kickPt[0], kickPt[1])
        const thetaKick = desiredHeadingTo(goal_now[0] - kickPt[0], goal_now[1] - kickPt[1])

        // Ground-plane distance can become noisy at close range, so pitch is
        // useful as a backup cue. It is not allowed to declare arrival by
        // itself: the 2026-06-27 hardware log showed pitch pinned down while
        // kickPt was still ~280mm away, causing a premature AT_KICK_POSE.
        const headPitchNow = robotPuPro.servoTargets()[5]
        const closeByDistance = distKick <= KICK_DIST_MM
        const closeByPitchAssist = ball_valid && headPitchNow >= NEAR_BALL_PITCH_DEG && distKick <= KICK_PITCH_ASSIST_DIST_MM
        const atKickPoint = closeByDistance || closeByPitchAssist
        const scoredNow = scored || (kickStarted && isScoredByVision(ball_now, goal_now))
        if (kickActive && nowMs - kickStartMs >= KICK_ACTION_MS) kickActive = false
        visionAcquireGoal = false
        visionPreferGoal = atKickPoint || kickActive

        if (scoredNow) {
            scored = true
            kickActive = false
            visionAcquireGoal = false
            visionPreferGoal = false
            walkSpeed = 0
            walkTurn = 0
            walkMode = 0
        } else if (kickActive && nowMs - kickStartMs < KICK_ACTION_MS) {
            // Keep the kick macro alive briefly, while vision keeps updating
            // in this same loop. If the ball was missed, the timeout below
            // releases back to approach/re-align using the latest estimate.
            walkSpeed = 0
            walkTurn = 0
            walkMode = 2
        } else if (!atKickPoint) {
            kickActive = false
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
            if (Math.abs(headingGoal) > ALIGN_HEADING_TOL) {
                kickActive = false
                walkSpeed = 1.0
                walkMode = 1
            } else {
                if (!kickActive) {
                    kickActive = true
                    kickStarted = true
                    kickStartMs = nowMs
                }
                walkSpeed = 0
                walkMode = 2
            }
        }

        if (DEBUG_FLAG) {
            serial.writeLine(`ball_now x=${ball_now[0]} y=${ball_now[1]}`)
            serial.writeLine(`goal_now x=${goal_now[0]} y=${goal_now[1]}`)
            serial.writeLine(`kickPt x=${kickPt[0]} y=${kickPt[1]} dist=${distKick} mode=${walkMode}`)
            if (scored) serial.writeLine("SCORED")
            if (walkMode == 2) serial.writeLine("AT_KICK_POSE")
        }
    } else {
        // Before both ball and goal have initialized the odometry-frame plan,
        // hold the body still and keep acquiring/searching with the camera.
        walkSpeed = 0
        walkTurn = 0
        walkMode = 0
        kickActive = false
        visionAcquireGoal = false
        visionPreferGoal = false
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
        robotPuPro.walk(BODY_WALK_SIGN * walkSpeed, walkTurn)
    } else if (walkMode == 1) {
        robotPuPro.walk(-BODY_WALK_SIGN * walkSpeed, walkTurn) // back up and turn to align with the goal
    } else {
        // walkMode == 2: aligned at the kick point. The extension's kick
        // action must be called repeatedly while the motion completes.
        robotPuPro.kick()
    }
    basic.pause(20)
})

/**
 * Stage 4 test (per stages-code/plan.md): Perform the Kick.
 *
 * Objective: identical to Stage 3 up through reaching and aligning at the
 * kick pose -- but instead of holding position once aligned (walkMode==2),
 * strike the ball: call robotPuPro.kick() repeatedly until the motion
 * completes, then drop the current ball lock so the planner re-detects and
 * re-approaches for another attempt (covers a missed kick). This is the only
 * behavioral change from stage3.ts; everything else (camera tracking,
 * latency compensation, Kalman filtering, arc steering, search, the stall
 * safety stop, the arrived-cycle debounce) is carried over unmodified.
 *
 * SELF-CONTAINED: paste this single file into MakeCode's main.ts, with the
 * robotPuPro extension attached (needed for locationArray()/servoStep()/
 * walk()/kick()/radio control, same as Stage 3).
 *
 * Built from professor-code/robotpu-soccer-mvp.ts + robotpu-localmap.ts +
 * robotpu-kalman-filter.ts + robotpu-viewpoint.ts, but those files have real
 * bugs/mismatches against the actual installed pxt-robotpu extension and
 * against each other. Fixes applied here (confirmed, not stylistic, carried
 * over from stage3.ts):
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
 * What this file deliberately does NOT do (out of scope for Stage 4):
 *   - No ball dynamics after the kick. The Kalman filter's small process
 *     noise (BALL_Q_POS/BALL_Q_VEL below) is tuned for a rolling/stationary
 *     ball before contact, not a struck one -- there's no point modeling
 *     post-kick motion here because the ball lock is dropped immediately
 *     once the kick finishes (see kickJustFinished below), forcing a fresh
 *     re-acquire/re-approach rather than continuing to track a moving ball.
 *   - No obstacle avoidance / A* (Stage 5). The LocalGrid below is only used
 *     for debug visualization of ball/goal/kick-point cells, matching
 *     robotpu-localmap.ts's original intent, not for path planning yet.
 *
 * How to verify:
 *   1. Show the robot both the ball and the goal. Watch serial for
 *      `ball_now`/`goal_now`/`kickPt` lines updating as you move either one.
 *   2. The robot should walk in a smooth curving approach toward the kick
 *      point (not a straight line + turn-in-place), then rotate to face the
 *      goal -- log line `AT_KICK_POSE` should print, then the robot performs
 *      the kick motion, then `KICK_DONE` prints once and the robot drops
 *      back into searching/approaching for another attempt.
 *   3. Confirm the kick triggers only once per approach -- no repeated
 *      kicking while still aligned in walkMode 2 (kickActive/kickJustFinished
 *      below exist specifically to prevent that).
 *   4. Manual override: pushing the remote's walk stick past the deadzone
 *      takes over from the autonomous controller immediately (same as Stage
 *      2/3), for safety/recovery while testing.
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

// Bug #17 (microbit-console-2026-06-28T01-32-08-449Z.txt and
// -01-29-20-096Z.txt, explicit user request): long stretches in both logs
// show BALL_TRACK reporting perfectly normal fresh readings while
// GOAL_REJECTED fires continuously at the exact same time, every single
// cycle -- the head is pointed down tracking the ball, so the SAME camera
// frame used for goal detection physically can't see the goal from down
// there. Worse, searchBall() below ONLY ever sweeps pitch in the level-to-
// down band (HEAD_PITCH_SEARCH_MIN..MAX = 90..135, see the comment on those
// constants) -- it never once points the head up toward level for the
// goal's sake, and nothing else in this file actively searches for the goal
// at all (the goal branch in trackPacket() never moves the head -- see its
// comment). So once the goal is lost, it can only ever be reacquired by
// accident, whenever ball-tracking happens to drift the head up on its own.
// User's explicit fix: ball search stays below level (the ball is on the
// ground, close), but add a second, separate search that sweeps the SAME
// yaw pattern at a fixed LEVEL pitch (HEAD_PITCH_CENTER) for the goal,
// which sits roughly at camera level rather than down at the robot's feet.
// Shares the yaw sweep state (scanStepIndex/scanFrameCounter/search_gain)
// with searchBall() below since the two are only ever called one at a time
// (see the planner loop) -- only the pitch target and log label differ.
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
        if (DEBUG_FLAG) serial.writeLine(`SEARCHING_BALL yaw=${nextYaw} pitch=${nextPitch}`)
        return
    }
    advanceSearchStep()
}

// Bug #17 above -- level-pitch yaw sweep for the goal, used only once the
// ball is already valid (so head pitch is free for the goal's sake) and the
// goal itself is missing. See the planner loop for the priority between
// this and searchBall().
function searchGoal() {
    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = SEARCH_PATTERN[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const nextYaw = clampL(HEAD_YAW_CENTER + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, HEAD_PITCH_CENTER, 8)
        if (DEBUG_FLAG) serial.writeLine(`SEARCHING_GOAL yaw=${nextYaw} pitch=${HEAD_PITCH_CENTER}`)
        return
    }
    advanceSearchStep()
}

function advanceSearchStep() {
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

// Ball: small process noise -- tuned for a rolling/stationary ball before
// contact. Post-kick ball dynamics are NOT modeled: the ball lock is dropped
// immediately once the kick finishes (see kickJustFinished below), so this
// filter never needs to track a ball that's actually moving from a strike.
const BALL_Q_POS = 5.0     // 5e-6 m^2 x1e6
const BALL_Q_VEL = 500.0   // 5e-4 m^2/s^2 x1e6
const BALL_R_FRESH = 1500.0 // 1.5e-3 m^2 x1e6

const ballKF = new Kalman2DConstVel()
const goalKF = new Kalman2DConstVel()
let lastKfMs = -1

// ---------------------------------------------------------------------------
// Kick point / heading helpers (from robotpu-localmap.ts / robotpu-soccer-mvp.ts)
// ---------------------------------------------------------------------------
// Bug #7 (microbit-console "2.txt"/"3.txt" pair, both 100% walkMode 0 start
// to finish -- AT_KICK_POSE never fires once in either log): in "2.txt" the
// robot is confirmed by the user to make real physical contact (it dribbles
// the ball into the goal with the ball between its legs) while distKick is
// STILL logging 100-160mm the entire time (e.g. dist=121.30, 126.95, 132.37,
// 144.64, 149.27...) -- it never gets anywhere near the old 60mm threshold.
// That's a calibration floor, not noise: there's a persistent ~100-150mm gap
// between computed distKick and true contact (odometry/camera-projection
// lag accumulating at close range), so a 60mm "arrived" threshold is simply
// unreachable in practice -- the robot always face-plants into the ball
// while still in approach mode instead of ever reaching the kick branch.
// Raised KICK_DIST_MM to match the observed real-contact band so "arrived"
// actually triggers. Backoff also dropped to 1cm per explicit request: the
// kick stance point should sit just behind the ball, with the ball always
// between that stance point and the goal (computeKickPoint() already builds
// it that way -- only the magnitude needed to shrink).
//
// Bug #8 (microbit-console-2026-06-27T21-09-40-430Z.txt, taken WITH
// KICK_DIST_MM=150): still not enough margin -- the close approach in that
// log reads dist=204.55 -> dist=181.83 (one 20ms planner cycle, never
// dropping below 150) and the very next reading jumps back UP to 205.68 and
// keeps climbing from there, while the user confirms the robot touched/
// pushed the ball through this exact stretch. Diagnosis at the time: a
// quadruped gait can't stop mid-stride, and ballKF lags a ball that's
// already being pushed, so distKick under-reports true closeness right
// where it matters most -- tried fixing this by raising KICK_DIST_MM to 250
// for more stopping margin.
//
// Reverted (explicit correction from the user): 250mm is the wrong
// direction entirely. The robot's kick has almost no reach -- it can only
// actually connect with the ball from ~1cm away. A 250mm "arrived" radius
// means the robot stops APPROACHING and starts the kick sequence while still
// ~25cm short of the ball, which just whiffs at thin air; it doesn't matter
// how much approach margin that leaves, the kick itself can't cover that gap.
// KICK_DIST_MM has to stay tight, matching the real kick range -- but that
// reopens bug #7/#8's problem: distKick's own ~100-200mm floor/lag means a
// tight threshold may rarely fire from distance alone, which is exactly why
// contactDetected (see CONTACT_* below, in the planner loop) exists now --
// it's a second, independent "arrived" signal based on actual physical
// resistance (commanded motion, ~zero real displacement) instead of trusting
// the laggy geometric estimate at sub-cm precision.
const KICK_BACKOFF_MM = 10   // 1cm behind the ball, ball stays between kick point and goal
const KICK_DIST_MM = 10      // tight on purpose -- matches the kick's real ~1cm reach, not approach stopping margin
const APPROACH_OFFSET_START_MM = 450 // raised with KICK_DIST_MM so the arc-in still starts well before the new stop distance
// Range (beyond stopDist_mm) over which updateControl()'s lead/lateral offset
// tapers back to 0 -- see the finalApproachTaper comment in updateControl()
// for the hardware-confirmed bug this fixes (bug #6).
const FINAL_STRAIGHT_RANGE_MM = 150
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
    // Confirmed backwards on hardware: robot veered away from the ball during
    // approach (turned left instead of heading toward it) and lost tracking
    // because of it -- flipped from -2.0.
    const kTurn = 2.0

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

    // Bug #6 (confirmed on hardware, microbit-console-2026-06-27T20-24-58-288Z.txt):
    // offsetGain rises toward 1 as dist shrinks toward 0 (it's "how far along
    // the approach are we", not "how far from the target"), so lead/lateral
    // were LARGEST exactly when closest to the kick point -- the lookahead
    // point used for heading could be up to 100mm to the side and 180mm past
    // a target that might only be ~150mm away. That log shows the robot's
    // heading swing ~90 degrees in a single step right as distKick bottomed
    // out near 150mm, then distKick climbed every cycle afterward and never
    // recovered -- the robot turned away and missed the ball instead of
    // walking the last bit straight in. Fixed with a second, independent
    // taper that collapses lead/lateral to 0 as dist approaches stopDist_mm,
    // so the final approach is a direct line at the target regardless of
    // what offsetGain (the long-range curve-in shaping) says.
    const finalApproachTaper = clampL((dist - stopDist_mm) / FINAL_STRAIGHT_RANGE_MM, 0.0, 1.0)
    const offsetGain = clampL(1.0 - dist / offsetStartDist_mm, 0.0, 1.0) * finalApproachTaper
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

basic.showString("4")
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

// Bug #12 (microbit-console-2026-06-27T22-25-40-464Z.txt): user reports the
// robot still walks off to one side even starting squarely on the kick line.
// That log shows BALL_TRACK reporting the literal SAME x_mm/y_mm (e.g.
// x_mm=-90 y_mm=485, later x_mm=-132 y_mm=493) across 30-100+ CONSECUTIVE
// fresh (VALID, non-STALE) packets, while the head yaw sweeps its entire
// physical range (46 to 135 degrees) during that same stretch. A real camera
// feed always has some frame-to-frame jitter even on a perfectly stationary
// ball -- an exact-repeat streak this long means the ESP32/I2C side is
// re-serving the same cached detection result without actually producing a
// fresh one, while still marking it VALID/non-STALE. Worse, lastYawByte (the
// head-tracking nudge) is read from this SAME stale packet and re-applied
// every cycle, so a frozen nonzero nudge keeps dragging the head (and thus
// the steering target) toward one side even though nothing in the scene
// actually moved -- explaining the persistent one-sided drift independent of
// the robot's actual starting alignment. Track how many consecutive fresh
// ball packets report the exact same raw (x_mm, y_mm); past this many,
// treat it as a non-advancing/stale detection and force the ball lost so
// search() recenters the head, instead of continuing to drive off it.
const FROZEN_BALL_CYCLE_LIMIT = 15
let frozenBallCycles = 0
let lastBallRawXY = [99999, 99999] // sentinel outside i16 range -- never matches a real first reading

// Bug #13 (microbit-console-2026-06-28T00-08-39-870Z.txt): after the first
// BALL_FROZEN forced the ball lost, search() swept the head through its
// ENTIRE physical range (yaw 45 to 135, every SEARCH_PATTERN step) for the
// rest of the log -- and every single packet still reported the exact same
// x_mm=-60 y_mm=355, never once different, regardless of where the camera
// was actually pointed. A cached-but-eventually-refreshing detector would
// produce a different (or no) reading once the head points somewhere a real
// ball isn't visible; identical output independent of camera orientation
// means the ESP32's own detection pipeline is stuck, not just slow to
// refresh -- ball_valid/searchBall() can never reach far enough to fix that,
// since neither one touches the ESP32 again once a reading already came
// back VALID/non-STALE. frozenBallCycles already keeps climbing for as long
// as the same stuck value keeps coming back (it only resets on a genuinely
// different reading) -- reuse it on a much longer fuse to retry the one
// lever available here: toggling the ball detection service off/on, the same
// call already used once at boot, to force the ESP32 to reinitialize.
// Checked with modulo (not a one-shot flag) so it keeps retrying
// periodically if a single toggle doesn't unstick it.
const BALL_FROZEN_RESET_CYCLE_LIMIT = 150 // ~3s -- well past one full SEARCH_PATTERN sweep

function trackPacket(p: Buffer) {
    const type = p[0]
    const flags = p[3]
    const count = p[4]
    const isValid = (flags & VALID) != 0 && count > 0
    const isStale = (flags & STALE) != 0
    const nowMs = input.runningTime()

    if (type == SOCCER_BALL) {
        // Bug #14 (microbit-console-2026-06-28T00-27-55-542Z.txt): user
        // observed the robot walk the right path and reach the kick point
        // correctly, but kickPt kept drifting hundreds of mm during/after
        // the kick itself (e.g. dist=345 at AT_KICK_POSE, climbing straight
        // to dist=754 by KICK_DONE, all while the body wasn't walking
        // anywhere -- kickActive already overrides walkMode in the actuator
        // loop). Once the robot is this close, the ball is in the head
        // camera's blind spot -- it physically can't see it -- so every
        // BALL_TRACK packet read during the kick is a bad close-range
        // reading, not a real update, and feeding it into ballKF just
        // corrupts the estimate for no benefit (the body isn't moving
        // toward anything while kicking). Freeze the ball entirely for the
        // duration of the kick: skip this packet outright, no Kalman
        // update, no head nudge, no staleness countdown. The planner loop's
        // kickJustFinished handling already drops ball_valid the instant
        // the kick completes, so normal tracking/searching resumes there --
        // nothing useful to track mid-kick anyway.
        if (kickActive) return
        const x_mm = i16(p, 6)
        const y_mm = i16(p, 8)
        const plausible = norm2L(x_mm, y_mm) <= MAX_DETECT_DIST_MM

        if (isValid && !isStale && plausible) {
            if (x_mm == lastBallRawXY[0] && y_mm == lastBallRawXY[1]) {
                frozenBallCycles += 1
            } else {
                frozenBallCycles = 0
                lastBallRawXY = [x_mm, y_mm]
            }
            if (frozenBallCycles >= BALL_FROZEN_RESET_CYCLE_LIMIT && frozenBallCycles % BALL_FROZEN_RESET_CYCLE_LIMIT == 0) {
                // Bug #13 above -- still stuck on this same reading long
                // after multiple force-lost/re-search cycles already had a
                // chance to find a fresh one. Try reinitializing the ESP32's
                // own detection pipeline instead of just waiting it out again.
                //
                // Escalated (microbit-console-2026-06-28T02-06-49-667Z.txt):
                // toggling SERVICE_SOCCER_BALL_DETECTION alone fired here but
                // the exact same frozen x_mm/y_mm kept coming back for the
                // rest of that run -- the hang is in image capture itself,
                // not just the ball-detection feature flag layered on top of
                // it, so cycling that one flag did nothing. Also bounce
                // SERVICE_IMAGE_CAPTURE to restart the underlying capture
                // pipeline, not just the sub-feature reading its output.
                setService(SERVICE_SOCCER_BALL_DETECTION, false)
                basic.pause(10)
                setService(SERVICE_IMAGE_CAPTURE, false)
                basic.pause(10)
                setService(SERVICE_IMAGE_CAPTURE, true)
                basic.pause(10)
                setService(SERVICE_SOCCER_BALL_DETECTION, true)
                if (DEBUG_FLAG) serial.writeLine("BALL_DETECTION_RESET -- toggling image capture + ball service, stuck on one reading too long")
            }
            if (frozenBallCycles >= FROZEN_BALL_CYCLE_LIMIT) {
                // Stale/non-advancing detection (bug #12 above) -- don't
                // drive the head or the body off a reading that hasn't
                // actually changed in this many cycles. Force lost; the
                // planner loop's `if (!ball_valid) searchBall()` recenters
                // the head on the next cycle.
                ball_valid = false
                if (DEBUG_FLAG) serial.writeLine(`BALL_FROZEN x_mm=${x_mm} y_mm=${y_mm} -- repeated identical reading, forcing lost`)
                return
            }

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
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)

            if (nextPitch <= HEAD_PITCH_OPERATING_MIN + PITCH_PIN_UP_MARGIN_DEG) {
                pitchPinnedUpCycles += 1
            } else {
                pitchPinnedUpCycles = 0
            }

            if (pitchPinnedUpCycles >= PITCH_PIN_UP_CYCLE_LIMIT) {
                // Pinned looking up for too long while supposedly tracking --
                // this is the runaway-up bug, not a real detection. Force
                // lost; the planner loop's `if (!ball_valid) searchBall()`
                // picks this up and recenters the head on the next cycle (see
                // below for why searchBall() isn't called inline here).
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

            // Raw detection coordinates -- previously only the resulting head
            // yaw/pitch were logged here (unlike the goal branch's GOAL_SEEN/
            // GOAL_REJECTED below), so a false-positive "ball" lock from the
            // camera's color threshold (vs. a real ball) was impossible to
            // distinguish from the log alone. Logging x_mm/y_mm gives the next
            // hardware capture something concrete to check the false lock
            // against (e.g. a suspiciously constant or implausible value).
            if (DEBUG_FLAG) serial.writeLine(`BALL_TRACK x_mm=${x_mm} y_mm=${y_mm} yaw=${nextYaw} pitch=${nextPitch}`)
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
            ball_valid = false
        }
        if (DEBUG_FLAG && isValid && !isStale && !plausible) {
            serial.writeLine(`BALL_REJECTED x_mm=${x_mm} y_mm=${y_mm} (implausible -- likely ceiling/wall noise)`)
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
// drive there. walkMode 2 ("aligned, at kick point") triggers the actual
// kick in the actuator loop below; this loop's only Stage-4-specific job is
// consuming kickJustFinished (set by the actuator loop once a kick attempt
// completes) to drop the ball lock and force a fresh re-approach.
// ---------------------------------------------------------------------------
let walkSpeed = 0
let walkTurn = 0
let walkMode = 0

// Cross-loop signal: the actuator loop sets this once a robotPuPro.kick()
// attempt has actually completed (see kickActive there). The planner loop
// consumes it below and drops the ball lock -- per stages-code/plan.md's
// Stage 4 description, "the loop falls back to re-detecting the ball/goal
// so it can re-approach for another attempt if the first kick missed."
let kickActive = false
let kickJustFinished = false

// Safety stop for a physically stuck robot (e.g. pushed against the goal/
// an obstacle -- this file still has no contact/obstacle sensor at all, by
// design, so nothing else here would ever notice on its own). Compares
// actual odometry displacement against commanded walk speed: if we're
// commanding real forward motion during approach but position isn't
// actually advancing, force-stop and drop the current ball lock so the
// planner re-searches instead of grinding against whatever it hit
// indefinitely. This also covers the case where the ball lock itself is a
// false positive (camera fixated on something that isn't really there,
// confirmed on hardware -- BALL_TRACK kept reporting a smoothly-tracked
// ball with no real ball in the scene) -- regardless of *why* the target
// is bad, "commanding motion with ~zero real displacement" is itself the
// unsafe condition worth stopping on.
const STALL_SPEED_THRESHOLD = 0.5
const STALL_DIST_EPS_MM = 5
const STALL_CYCLE_LIMIT = 50 // ~1s at this loop's 20ms cadence
let stallCycles = 0
let stallCheckPose_mm = [0, 0]

// Bug #9 (companion to the KICK_DIST_MM revert above): the kick's real reach
// is ~1cm, so KICK_DIST_MM has to stay tight -- but distKick's own ~100-200mm
// floor/lag (bug #7/#8) means a tight threshold may rarely or never satisfy
// `distKick <= KICK_DIST_MM` even once the robot is genuinely touching the
// ball. Don't trust the geometric estimate alone at this precision: reuse the
// same physical signal as the STALL check above (commanded forward motion,
// ~zero real displacement -- i.e. something is physically resisting the
// robot's legs), but on a much shorter fuse and gated to only count once
// already plausibly close, so it reads as "I just hit the ball" rather than
// "I'm stuck/lost" (that's still STALL_CYCLE_LIMIT's job, for the cases this
// gate excludes -- e.g. genuinely walking into a wall from across the field).
const CONTACT_GATE_DIST_MM = 400
const CONTACT_DIST_EPS_MM = 5
const CONTACT_CYCLE_LIMIT = 6 // ~120ms -- short enough to catch contact almost immediately, unlike STALL_CYCLE_LIMIT's ~1s
let contactCycles = 0
let contactCheckPose_mm = [0, 0]

// Debounce for the "arrived at kick point" transition (mode 0 -> 1/2).
// Hardware logs show ball_now/goal_now both flipping sign in a single cycle
// at exactly these transitions (e.g. y=+298 -> y=-169 for the ball AND
// y=+831 -> y=-1123 for the goal, same cycle) -- too coincidental to be two
// independent Kalman filters drifting; the only thing they share is the
// live poseNow used by odomToNow(), so a single bad/glitched pose sample
// can make distKick read artificially low for one cycle and trigger a
// premature stop well before the robot actually closed the distance.
// Requiring the arrival condition to hold for several consecutive cycles
// makes a one-frame glitch unable to trigger the transition by itself,
// regardless of whether the glitch is confirmed to be in locationArray()
// or somewhere else.
const ARRIVED_CYCLE_LIMIT = 5
let arrivedCycles = 0

// Debounce for nearBallByPitch specifically (bug #3): the head saturates at
// HEAD_PITCH_MAX as soon as the ball gets low enough in frame, which happens
// well before the robot is actually close enough to kick -- a single pinned
// reading isn't a trustworthy "ball is at my feet" signal by itself. Require
// it to hold for many consecutive cycles (the head has nowhere else to look,
// not just a momentary saturation) before it's allowed to short-circuit the
// approach the way a single reading used to.
const NEAR_BALL_PITCH_CYCLE_LIMIT = 25 // ~0.5s at this loop's 20ms cadence
let pitchNearCycles = 0

// Bug #5 (confirmed on hardware, microbit-console-2026-06-27T19-40-59-450Z.txt):
// nearBallByPitch's whole premise -- "pitch pinned near the floor limit means
// the ball is at the robot's feet" -- doesn't hold on this hardware. That log
// shows pitch sustained above NEAR_BALL_PITCH_DEG (125) for 30+ consecutive
// cycles (130-133 deg) while distKick sat at 500-550mm the whole time, so
// pitchConfirmed alone (the `|| pitchConfirmed` below) declared "arrived" and
// flipped walkMode to 1/2 hundreds of mm short of the ball -- the robot
// backed up/turned away (walkMode 1) or tried to kick (walkMode 2) without
// ever actually reaching it, matching the user's "turned left, missed the
// ball" report. Gate the pitch override behind a generous-but-real distance
// bound instead of letting it fire unconditionally: it can only shortcut the
// distance check once the robot is already plausibly close (where the
// camera's ground-plane math is known to be unreliable -- the original
// reason this existed), not from across the whole field.
const NEAR_BALL_PITCH_DIST_GATE_MM = 200

// Bug #18 (microbit-console-2026-06-28T01-29-20-096Z.txt): kickPt was logged
// growing into the tens of thousands of mm (dist=60000+) and never recovered
// for the rest of that run -- the robot just kept backing up/re-approaching
// (walkMode 3 <-> 0) chasing a point many dozens of meters away on a field
// that's ~1-2m across. Traced to poseNow itself reading an unchanging,
// clearly-wrong value (e.g. theta=11.33) for dozens of consecutive cycles --
// Stage 3 bug #19's pose freeze, still unresolved at the extension level --
// which corrupts odomToNow()'s re-projection of BOTH ball_now and goal_now
// (goal_now exploded the same way even off fresh GOAL_SEEN updates, so this
// isn't ball-coasting-specific). Since nothing here can fix locationArray()
// itself, treat an exploded kickPt as a sign the current ball/goal lock is
// built on a corrupted pose sample and force a full reset rather than
// wandering after a target that can never realistically be reached.
const PLAUSIBLE_KICKPT_DIST_MM = 3000 // generous vs. the ~1-2m real field, but catches a multi-meter blowup

// Bug #6 recovery (requested after microbit-console-2026-06-27T20-24-58-288Z.txt
// showed the robot overshoot the kick point, turn away, and just keep walking
// forward into a wall instead of stopping or correcting): once distKick grows
// for many consecutive cycles while still approaching (walkMode 0), that's a
// miss, not normal jitter -- the fix to updateControl()'s lead/lateral taper
// above should make misses much rarer, but this is the requested fallback for
// whenever one still happens. walkMode 3 backs the robot away from the missed
// kick point while re-orienting to face it, then hands back to the normal
// walkMode 0 approach for another attempt, instead of plowing forward blind.
const MISS_DIST_EPS_MM = 20 // must grow by at least this much in one cycle to count as "moving away", not measurement noise
const MISS_CYCLE_LIMIT = 15 // ~0.3s of sustained moving-away before declaring a miss
let missCycles = 0
let prevDistKick = -1

const BACKUP_CYCLE_LIMIT = 50 // ~1s of backing away before retrying the approach
const BACKUP_SPEED = -1.5
let backupCycles = 0

basic.forever(function () {
    const packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackPacket(packet)
    } else if (DEBUG_FLAG) {
        serial.writeLine(`I2C_ERR len=${packet.length}`)
    }

    if (kickJustFinished) {
        kickJustFinished = false
        ball_valid = false
        // Bug #16 above: the walk gate below no longer depends on
        // ball_valid alone (it also accepts a coasting ballKF.inited
        // estimate through blind-spot dropouts) -- so ball_valid=false by
        // itself is no longer enough to stop the body from immediately
        // resuming on the stale pre-kick ball position. Force ballKF back to
        // uninited so the next real detection does a hard reset() (fresh
        // position, fresh covariance) instead of blending with a position
        // that's now wrong (the ball just got struck and moved).
        ballKF.inited = false
        if (DEBUG_FLAG) serial.writeLine("KICK_DONE -- dropping ball lock to re-approach")
    }

    const nowMs = input.runningTime()
    const poseNow = robotPuPro.locationArray() // [x_mm, y_mm, theta_deg]

    // Bug #14 above -- don't let the ball time out or trigger a search while
    // the kick itself is in progress. trackPacket() already skips all ball-
    // packet processing during kickActive, so lastBallSeenMs stops advancing
    // too -- without this gate, the kick's own duration would eventually
    // trip the staleness timeout and start dragging the head into search
    // mid-kick for no reason.
    if (ball_valid && !kickActive && nowMs - lastBallSeenMs >= BALL_LOST_TIMEOUT_MS) ball_valid = false
    if (goal_valid && nowMs - lastGoalSeenMs >= GOAL_LOST_TIMEOUT_MS) goal_valid = false

    // Centralized here instead of inside trackPacket()'s branches: both ball
    // and goal detection services are enabled and the camera interleaves
    // packet types, so a search call nested inside one branch simply
    // doesn't run on a cycle where the other packet type happens to arrive.
    // Calling it unconditionally here makes search progress independent of
    // which packet type happened to arrive.
    //
    // Bug #17 (see searchGoal() above): ball-finding still takes priority
    // when both are missing (matches the previous behavior, and nothing
    // useful can be computed without the ball ever being seen at least
    // once) -- but once the ball is valid and the goal specifically is the
    // one missing, actively sweep level for the goal instead of leaving the
    // head wherever ball-tracking put it (often pointed down, where the
    // goal physically can't be seen -- this was silently true the entire
    // time before, explaining the long GOAL_REJECTED streaks alongside
    // perfectly normal BALL_TRACK in the user's logs).
    if (!kickActive) {
        if (!ball_valid) {
            searchBall()
        } else if (!goal_valid) {
            searchGoal()
        }
    }

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
        const goal_meas_O = camToOdom(goal_cam2D_mm, goal_head_yaw_deg, goal_pose_mm_deg)
        goalKF.update(goal_meas_O[0], goal_meas_O[1], GOAL_R_FRESH)
    }
    if (haveBallMeas) {
        const ball_meas_O = camToOdom(ball_cam2D_mm, ball_head_yaw_deg, ball_pose_mm_deg)
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

    // Bug #16 (microbit-console-2026-06-28T01-00-55-670Z.txt was a different,
    // worse experiment -- locking the kick point entirely from the first
    // detection and walking on odometry alone. That fully exposed Stage 3's
    // pose freeze-then-jump bug #19 with zero vision correction for the
    // whole approach, AND lost the continuous head-down tracking that used
    // to counteract gait vibration, so the head visibly drifted up while
    // walking. Reverted.
    //
    // The actual, narrower problem this user keeps reporting (this session,
    // repeatedly): the closer the robot gets, the more often the ball is in
    // the head camera's blind spot, so `ball_valid` goes false (after
    // BALL_LOST_TIMEOUT_MS) even though the ball hasn't moved -- and once
    // that happens, this gate used to require BOTH ball_valid && goal_valid,
    // stopping the body outright. But ball_now/ballKF.pos() are already
    // computed unconditionally above via Kalman PREDICT every cycle
    // regardless of ball_valid (see the comment above ball_now) -- a
    // stationary ball has ~0 velocity in this filter, so the predicted
    // position barely moves even through a long blind-spot stretch with no
    // fresh measurement. Only require goal_valid (the goal isn't subject to
    // this blind-spot problem) plus ballKF.inited (we've gotten at least one
    // real ball fix this approach) to keep walking -- ball_valid itself is
    // no longer a hard requirement to walk, only to feed fresh measurements
    // into the filter when available.
    if (goal_valid && ballKF.inited) {
        const kickPt = computeKickPoint(ball_now, goal_now)
        const idxK = grid.index(kickPt[0], kickPt[1])
        grid.set(idxK[0], idxK[1], 4)

        const distKick = norm2L(kickPt[0], kickPt[1])

        // Bug #18 above: a kickPt this far away can only come from a
        // corrupted pose sample, not a real approach -- drop everything and
        // force a fresh re-acquisition rather than chasing it.
        if (distKick > PLAUSIBLE_KICKPT_DIST_MM) {
            ball_valid = false
            goal_valid = false
            ballKF.inited = false
            goalKF.inited = false
            walkSpeed = 0
            walkTurn = 0
            walkMode = 0
            arrivedCycles = 0
            pitchNearCycles = 0
            missCycles = 0
            backupCycles = 0
            prevDistKick = -1
            contactCycles = 0
            if (DEBUG_FLAG) serial.writeLine(`KICKPT_IMPLAUSIBLE dist=${distKick} -- pose/Kalman projection blew up, forcing full reset`)
        } else {
        const thetaKick = desiredHeadingTo(goal_now[0] - kickPt[0], goal_now[1] - kickPt[1])

        // Ground-plane distance from a single pitched camera is only reliable
        // far from the robot -- near the camera's blind spot (head pitched
        // down toward the robot's own feet) small pitch errors blow up into
        // large distance errors, so the geometric distKick estimate can stay
        // stuck well above KICK_DIST_MM even as the ball is right there. Use
        // head pitch itself as a physical "ball is right here" cue once it's
        // pinned near its downward limit (HEAD_PITCH_MAX -- corrected
        // convention: low pitch is up, high pitch is down), instead of
        // trusting distance alone -- BUT (bug #3, confirmed on hardware) a
        // single pinned reading fires as soon as the ball gets low enough in
        // frame, well before the robot is actually close, so require it to
        // hold for many consecutive cycles before it counts (pitchNearCycles).
        // Bug #16 above: this used to also require ball_valid, but that's
        // exactly the flag that goes false during a blind-spot dropout --
        // blocking this signal right when it would matter most. The head
        // still holds its last commanded pitch when ball_valid drops (no
        // head movement happens in trackPacket()'s lost branch), so a
        // pinned-down reading from right before the dropout remains a valid
        // "ball is right here" signal; ballKF.inited (guaranteed true to
        // even reach this block now) is enough context to trust it.
        const headPitchNow = robotPuPro.servoTargets()[5]
        const nearBallByPitch = headPitchNow >= NEAR_BALL_PITCH_DEG
        pitchNearCycles = nearBallByPitch ? pitchNearCycles + 1 : 0
        const pitchConfirmed = pitchNearCycles >= NEAR_BALL_PITCH_CYCLE_LIMIT

        // Bug #9 (see CONTACT_* above): a third, independent "arrived" signal
        // based on physical resistance -- commanded forward motion with ~zero
        // real displacement, gated to only count once already plausibly
        // close. distKick alone can't resolve the ~1cm precision the kick
        // actually needs (its floor/lag is ~100-200mm, confirmed on hardware
        // -- see bug #7/#8), so a tight KICK_DIST_MM needs this as a backstop.
        const contactGated = walkMode == 0 && distKick <= CONTACT_GATE_DIST_MM
        if (contactGated) {
            const movedSinceLastCycle = norm2L(poseNow[0] - contactCheckPose_mm[0], poseNow[1] - contactCheckPose_mm[1])
            contactCycles = movedSinceLastCycle < CONTACT_DIST_EPS_MM ? contactCycles + 1 : 0
        } else {
            contactCycles = 0
        }
        contactCheckPose_mm = [poseNow[0], poseNow[1]]
        const contactDetected = contactCycles >= CONTACT_CYCLE_LIMIT

        // Bug #11 (microbit-console-2026-06-27T22-14-49-072Z.txt): walkMode
        // never left 0 despite distKick visibly dipping to 143mm (well inside
        // both NEAR_BALL_PITCH_DIST_GATE_MM and CONTACT_GATE_DIST_MM) -- the
        // pose was frozen for ~16 straight cycles (Stage 3 bug #19, still
        // unresolved at the extension level), so contactCycles correctly hit
        // CONTACT_CYCLE_LIMIT and contactDetected fired right at dist=368 --
        // but distKick crossed back above its gates the very next cycle,
        // which zeroed arrivedCycles before it ever reached ARRIVED_CYCLE_LIMIT.
        // The bug is requiring pitchArrived/contactDetected to ALSO survive a
        // separate 5-cycle arrivedCycles streak on top of their OWN internal
        // sustained-confirmation debounce (pitchNearCycles>=25,
        // contactCycles>=6) -- redundant, and fatal right at the target where
        // distKick is naturally volatile (the entire reason these two signals
        // exist instead of trusting distance alone). Only the raw-distance
        // path still needs arrivedCycles (that's the one actually vulnerable
        // to a single glitched pose sample, per the debounce's original
        // motivation); pitchArrived/contactDetected now count as arrived the
        // instant they fire.
        const distCloseEnough = distKick <= KICK_DIST_MM
        const pitchArrived = pitchConfirmed && distKick <= NEAR_BALL_PITCH_DIST_GATE_MM
        arrivedCycles = distCloseEnough ? arrivedCycles + 1 : 0
        const arrived = arrivedCycles >= ARRIVED_CYCLE_LIMIT || pitchArrived || contactDetected

        if (walkMode == 3) {
            // Missed last attempt -- keep backing away from the (re-detected,
            // possibly shifted) kick point while turning to face it, rather
            // than walking straight back in immediately (that's the approach
            // that just missed). Once backed off far enough, hand back to the
            // normal walkMode 0 approach below for a fresh attempt.
            backupCycles += 1
            const headingToKick = desiredHeadingTo(kickPt[0], kickPt[1])
            walkTurn = clampL(TURN_GAIN * headingToKick, -0.8, 0.8)
            walkSpeed = BACKUP_SPEED
            if (backupCycles >= BACKUP_CYCLE_LIMIT) {
                backupCycles = 0
                missCycles = 0
                prevDistKick = -1
                arrivedCycles = 0
                walkMode = 0
            }
        } else if (!arrived) {
            const ctrl = updateControl(
                { x: 0, y: 0, theta: 0 },
                { x: kickPt[0], y: kickPt[1], theta: thetaKick },
                APPROACH_OFFSET_START_MM,
                KICK_DIST_MM
            )
            walkSpeed = ctrl[0]
            walkTurn = ctrl[1]
            walkMode = 0

            // Bug #6: a real miss shows up as distKick growing for many
            // consecutive cycles while still in approach mode -- not the
            // occasional single-cycle jitter already tolerated elsewhere in
            // this file. prevDistKick resets to -1 (skipping the check for
            // one cycle) whenever this path wasn't run last cycle, so a mode
            // switch or a fresh ball/goal lock can't be mistaken for a miss.
            //
            // Bug #10 (microbit-console-2026-06-27T21-09-40-430Z.txt): distKick
            // climbed for a long stretch overall but wasn't strictly
            // monotonic every single 20ms cycle -- occasional non-growing
            // cycles (sign flips/jitter as ball or goal crossed the robot's
            // heading) kept wiping missCycles straight back to 0 before it
            // ever reached MISS_CYCLE_LIMIT, so walkMode never reached 3
            // despite a clear, sustained miss in that log. Only clear the
            // streak outright on a cycle that's genuinely closing in (a real
            // miss never looks like that, even briefly); a flat/noisy cycle
            // decays it by one instead of zeroing it, so the streak survives
            // occasional jitter without masking real recovery.
            if (prevDistKick >= 0 && distKick > prevDistKick + MISS_DIST_EPS_MM) {
                missCycles += 1
            } else if (prevDistKick >= 0 && distKick < prevDistKick - MISS_DIST_EPS_MM) {
                missCycles = 0
            } else {
                missCycles = Math.max(0, missCycles - 1)
            }
            if (missCycles >= MISS_CYCLE_LIMIT) {
                missCycles = 0
                backupCycles = 0
                walkMode = 3
                if (DEBUG_FLAG) serial.writeLine("KICK_POINT_MISSED -- backing up to retry")
            }
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
        prevDistKick = distKick

        if (DEBUG_FLAG) {
            serial.writeLine(`pose x=${poseNow[0]} y=${poseNow[1]} theta=${poseNow[2]}`)
            serial.writeLine(`ball_now x=${ball_now[0]} y=${ball_now[1]}`)
            serial.writeLine(`goal_now x=${goal_now[0]} y=${goal_now[1]}`)
            serial.writeLine(`kickPt x=${kickPt[0]} y=${kickPt[1]} dist=${distKick} mode=${walkMode}`)
            if (walkMode == 2) serial.writeLine("AT_KICK_POSE")
        }
        }
    } else {
        // Goal not visible yet, or the ball has never been seen at all this
        // approach (bug #16 above): don't walk. Once ballKF has a real fix
        // and the goal is valid, losing ball_valid alone (e.g. a blind-spot
        // dropout) no longer falls into this branch -- see the gate above.
        walkSpeed = 0
        walkTurn = 0
        walkMode = 0
        arrivedCycles = 0
        pitchNearCycles = 0
        missCycles = 0
        backupCycles = 0
        prevDistKick = -1
        contactCycles = 0
    }

    // Stall check: only meaningful while actively driving forward in
    // approach mode (walkMode 0) -- walkMode 1/2/3 either back up
    // (deliberately, for alignment or a missed-kick retry), hold, or kick on
    // purpose, none of which should trip this.
    if (walkMode == 0 && Math.abs(walkSpeed) > STALL_SPEED_THRESHOLD) {
        const movedDist = norm2L(poseNow[0] - stallCheckPose_mm[0], poseNow[1] - stallCheckPose_mm[1])
        stallCycles = movedDist < STALL_DIST_EPS_MM ? stallCycles + 1 : 0
    } else {
        stallCycles = 0
    }
    stallCheckPose_mm = [poseNow[0], poseNow[1]]

    if (stallCycles >= STALL_CYCLE_LIMIT) {
        stallCycles = 0
        ball_valid = false
        walkSpeed = 0
        walkTurn = 0
        if (DEBUG_FLAG) serial.writeLine("STALL_DETECTED -- forcing ball lost, stopping")
    }

    basic.pause(20)
})

// ---------------------------------------------------------------------------
// Actuator loop. Remote stick (if pushed) always overrides the autonomous
// controller, same safety/recovery behavior as Stage 2/3.
// ---------------------------------------------------------------------------
basic.forever(function () {
    // Bug #4 (confirmed on hardware, microbit-console-2026-06-27T-stage4-2.txt):
    // the planner recomputes walkMode from scratch every single cycle purely
    // off headingGoal's instantaneous value (no hysteresis), and headingGoal
    // jitters across the 0.25 threshold from ordinary sensor/Kalman noise --
    // so walkMode flickered 2->1->2->1 every 2-3 cycles (~40-60ms) in that
    // log, NEVER once holding walkMode==2 long enough for robotPuPro.kick()'s
    // multi-step boxingStates motion to reach a strike position. Confirmed by
    // the total absence of "KICK_DONE" anywhere in that log despite 9 separate
    // AT_KICK_POSE attempts -- the kick was started and abandoned every time,
    // explaining the user's "kick is small" report (the leg barely begins
    // moving before walkMode==1 takes back over and calls walk() instead).
    // Fixed by making a started kick stick here in the actuator loop: once
    // kickActive is true, keep calling kick() every cycle regardless of what
    // the planner's walkMode does next, until kick() itself reports done.
    if (remoteWalkSpeed != 0) {
        robotPuPro.walk(remoteWalkSpeed, remoteWalkTurn)
    } else if (walkMode == 2 || kickActive) {
        // robotPuPro.kick() must be called repeatedly to complete the motion
        // (per pxt-robotpu/main.ts's own doc comment: "Returns 0 when the
        // kick is complete. Call repeatedly in a loop until it returns 0.").
        // A 0 return only counts as "finished" if a prior cycle's call was
        // already mid-kick (otherwise every cycle's first call would look
        // like an instant completion).
        const kickMd = robotPuPro.kick()
        if (kickMd != 0) {
            kickActive = true
        } else if (kickActive) {
            kickActive = false
            kickJustFinished = true
        }
    } else if (walkMode == 0) {
        robotPuPro.walk(walkSpeed, walkTurn)
    } else if (walkMode == 3) {
        // Missed-kick-point recovery: the planner already set walkSpeed to a
        // signed BACKUP_SPEED (negative) for this state, unlike walkMode 1
        // below -- pass it straight through instead of negating it again.
        robotPuPro.walk(walkSpeed, walkTurn)
    } else {
        // walkMode == 1, and no kick in progress: back up and turn to align with the goal
        robotPuPro.walk(-walkSpeed, walkTurn)
    }
    basic.pause(20)
})

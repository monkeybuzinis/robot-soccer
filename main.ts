/**
 * Stage 2 test (per plan.txt): Odometry Tracking & Active Sensor Searching.
 *
 * Objective: verify the robot's own pose updates correctly as it walks, and
 * that it actively sweeps its head to reacquire the ball once lost.
 *
 * NOTE on plan.txt's file list for this stage ("robotpu-odometry (1).ts"):
 * that file's leftStep()/rightStep() SE(2) class is NOT used here. I checked
 * the real pxt-robotpu extension source -- it already has its own internal
 * Odometry class, and every call to robotPuPro.walk() updates it
 * automatically (no event hook to attach to; "onWalkEvent" does not exist).
 * So instead of reimplementing odometry, this file just reads the pose the
 * extension is already tracking via robotPuPro.locationArray().
 *
 * Goal detection/filtering is deliberately NOT in this file -- that's a
 * separate, still-open problem (see main.ts's previous version / the
 * z_mm probe) and isn't part of what Stage 2 needs to verify. Only ball
 * detection is used here, to drive the "lost -> search" behavior.
 *
 * SELF-CONTAINED: paste this single file into MakeCode's main.ts. Attach the
 * robotPuPro extension this time (Stage 1 didn't need it; this stage does,
 * for locationArray()/servoStep()/walk()/radio control).
 *
 * How to verify:
 *   1. Pose tracking: drive the robot forward a few steps with your remote
 *      (radio, same as the other RobotPU files) and watch serial for
 *      `POSE x_mm=... y_mm=... theta_deg=...` updating smoothly, without
 *      freezing or throwing errors, as you walk and turn.
 *   2. Active search: show the ball to the camera (head should track it,
 *      `BALL_TRACK` lines update), then hide it. After LOST_TIMEOUT_MS the
 *      head should start sweeping through SEARCH_PATTERN on its own
 *      (`SEARCHING` lines), and re-lock immediately once the ball reappears.
 */

// ---------------------------------------------------------------------------
// I2C protocol / packet layout (from robotpu-i2c-cam.ts) -- ball only for now
// ---------------------------------------------------------------------------
const MUX_ADDR = 112  // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

const SOCCER_BALL = 0x04
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
// Head servo range: pxt-robotpu's PCB.servoStep() clamps to an ABSOLUTE
// [0,179] degree range, with 90 = looking straight ahead (confirmed against
// the real extension source) -- not a small offset around 0.
// ---------------------------------------------------------------------------
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45

const SCAN_WAIT_FRAMES = 25
const DEBUG_FLAG = true

function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
}

// ---------------------------------------------------------------------------
// Active head-scan search pattern (from robotpu-search-soccer.js), tuned so
// the search stays pointed at the floor instead of the ceiling/walls.
//
// Previously pitch offsets were applied cumulatively on top of the *live*
// servo position and scaled by search_gain (which grows up to 4x), so over a
// few scan cycles pitch could drift all the way to its +-45 deg clamp and
// stare at the ceiling/wall -- exactly what was observed on hardware. Now
// pitch is always an ABSOLUTE target close to a fixed ground-biased center,
// never cumulative and never gain-scaled, so it physically cannot run away.
// Yaw keeps the old cumulative/gain-scaled behavior since a wide left-right
// sweep across the field is what we actually want there.
//
// HEAD_PITCH_GROUND_BIAS's sign is unverified on real hardware -- if the head
// tilts UP instead of down during search, flip this to a negative value.
// ---------------------------------------------------------------------------
const HEAD_PITCH_GROUND_BIAS = 15
const HEAD_PITCH_SEARCH_CENTER = HEAD_PITCH_CENTER + HEAD_PITCH_GROUND_BIAS
const HEAD_PITCH_SEARCH_SPAN = 8
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
// Boot sequence: remote control + camera mux/service enable
// ---------------------------------------------------------------------------
robotPuPro.setChannel(166)
robotPuPro.setServoTrim(-5, 0, -5, 0, -8, 0)

// ---------------------------------------------------------------------------
// Remote-control walk is driven directly here instead of through
// runKeyValueCommand(). Reason: that path only works by setting gst=5
// (joystick mode) for the extension's own background stateMachine() to pick
// up -- but this script's other loops call servoStep()/setModeVar() every
// 20ms, which forces gst back to API (6) on (effectively) every tick, and
// API has no dispatch entry in stateFuncDict (pxt-robotpu/robotpu.ts). So
// gst=5 never survives long enough for joystick()'s walk() call to run,
// and locationArray() never changes -- confirmed against hardware logs
// showing pose frozen at (0,0,0) even while manually walking the robot.
// Calling robotPuPro.walk() directly (same call path joystick() uses
// internally) runs under the API mode this script is already forcing, so
// it always executes and updates odometry, regardless of gst races.
// ---------------------------------------------------------------------------
const REMOTE_FWD_SPEED = 3   // matches pxt-robotpu's internal fwdSpeed default
const REMOTE_BWD_SPEED = -2  // matches pxt-robotpu's internal bwdSpeed default
const REMOTE_STICK_DEADZONE = 0.2
let remoteWalkSpeed = 0
let remoteWalkTurn = 0

radio.onReceivedString(function (receivedString: string) {
    robotPuPro.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name: string, value: number) {
    if (DEBUG_FLAG) serial.writeLine(`RADIO_RX name=${name} value=${value}`)
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

// Repeatedly calling walk() is required to keep walking and to let each gait
// step complete (it returns 1 mid-step, 0 once a step finishes and odometry
// updates) -- same contract as the WALK_TEST trigger above.
basic.forever(function () {
    if (remoteWalkSpeed != 0) {
        const ret = robotPuPro.walk(remoteWalkSpeed, remoteWalkTurn)
        if (DEBUG_FLAG) serial.writeLine(`REMOTE_WALK speed=${remoteWalkSpeed} turn=${remoteWalkTurn} ret=${ret}`)
    }
    basic.pause(20)
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

// ---------------------------------------------------------------------------
// Explicit walk-step trigger for Stage 2 verification.
//
// Calling robotPuPro.servoStep()/setModeVar() anywhere (the head-tracking and
// search loops below do this every 20ms) forces gst=Mode.API every time. The
// extension's internal stateMachine() only has dispatch entries for
// gst -4..5 (see pxt-robotpu/robotpu.ts stateFuncDict) -- API (6) has none,
// so it silently does nothing. That means gst never stays at 5 (joystick)
// long enough for the remote's walk stick to actually run a gait step, so
// locationArray() can never change while these loops are active. Driving
// walk() directly from script code (same call path joystick() uses
// internally) sidesteps that conflict entirely.
// ---------------------------------------------------------------------------
const WALK_TEST_STEPS = 3
const WALK_TEST_SPEED = 2
let walkTestRunning = false

input.onGesture(Gesture.Shake, function () {
    if (walkTestRunning) return
    walkTestRunning = true

    const before = robotPuPro.locationArray()
    serial.writeLine(`WALK_TEST start pose x_mm=${before[0]} y_mm=${before[1]} theta_deg=${before[2]}`)

    let stepsDone = 0
    while (stepsDone < WALK_TEST_STEPS) {
        const ret = robotPuPro.walk(WALK_TEST_SPEED, 0)
        if (ret == 0) {
            stepsDone += 1
            const loc = robotPuPro.locationArray()
            serial.writeLine(`WALK_TEST step ${stepsDone}/${WALK_TEST_STEPS} pose x_mm=${loc[0]} y_mm=${loc[1]} theta_deg=${loc[2]}`)
        }
        basic.pause(20)
    }

    robotPuPro.setModeVar(robotPuPro.Mode.Rest)
    const after = robotPuPro.locationArray()
    serial.writeLine(`WALK_TEST done pose x_mm=${after[0]} y_mm=${after[1]} theta_deg=${after[2]}`)
    walkTestRunning = false
})

basic.showString("2")
pins.i2cWriteNumber(MUX_ADDR, 0x0F, NumberFormat.Int8LE, false)
basic.pause(2000)

basic.forever(function () {
    setService(SERVICE_IMAGE_CAPTURE, true)
    basic.pause(10)
    setService(SERVICE_FACE_DETECTION, false)
    basic.pause(10)
    setService(SERVICE_SOCCER_BALL_DETECTION, true)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, true) // left enabled so Stage 1's gate work isn't lost, just not used here
    basic.pause(30000)
})

// ---------------------------------------------------------------------------
// Pose readout: confirms robotPuPro.locationArray() updates as you walk it
// manually with the remote, without freezing or erroring.
// ---------------------------------------------------------------------------
basic.forever(function () {
    const loc = robotPuPro.locationArray() // [x_mm, y_mm, theta_deg]
    serial.writeLine(`POSE x_mm=${loc[0]} y_mm=${loc[1]} theta_deg=${loc[2]}`)
    basic.pause(250)
})

// ---------------------------------------------------------------------------
// Ball tracking + active search-when-lost.
//
// Previously this only refreshed "last seen" on ANY packet with VALID set,
// including STALE ones -- but the camera keeps sending VALID+STALE packets
// with the ball's last-known coordinates even after the ball is physically
// removed, so the old ms-based timeout never fired (lastBallSeenTime kept
// getting refreshed by stale data forever). Now a fresh, non-stale packet is
// the only thing that resets the miss counter; STALE/missing/wrong-type
// packets all count as a miss. After BALL_STALE_CYCLE_LIMIT consecutive
// misses, ballValid is forced false and search takes over immediately,
// regardless of how many more stale repeats keep arriving.
// ---------------------------------------------------------------------------
const BALL_STALE_CYCLE_LIMIT = 15 // consecutive non-fresh loop cycles (~20ms each below) before declaring the ball lost
let ballMissCycles = 0
let ballValid = false

basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length != SIZE) {
        ballMissCycles += 1
        if (ballMissCycles >= BALL_STALE_CYCLE_LIMIT) ballValid = false
        if (!ballValid) searchBall()
        basic.pause(20)
        return
    }

    const type = packet[0]
    const flags = packet[3]
    const count = packet[4]
    const isBallPacket = type == SOCCER_BALL && (flags & VALID) && count > 0
    const isStale = (flags & STALE) != 0

    if (isBallPacket && !isStale) {
        ballMissCycles = 0
        ballValid = true
    } else {
        ballMissCycles += 1
        if (ballMissCycles >= BALL_STALE_CYCLE_LIMIT) ballValid = false
    }

    if (ballValid && isBallPacket) {
        search_gain = 1.0
        const yawByte = i8(packet[16])
        const pitchByte = i8(packet[17])
        const staleScale = isStale ? 0.3 : 1.0

        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const liveYaw = robotPuPro.servoTargets()[4]
        const livePitch = robotPuPro.servoTargets()[5]
        const nextYaw = clampL(liveYaw + yawByte * staleScale * 0.08, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(livePitch + pitchByte * staleScale * 0.08, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)

        if (DEBUG_FLAG) serial.writeLine(`BALL_TRACK yaw=${nextYaw} pitch=${nextPitch} stale=${isStale ? 1 : 0} missCycles=${ballMissCycles}`)
    } else {
        searchBall()
    }

    basic.pause(20)
})

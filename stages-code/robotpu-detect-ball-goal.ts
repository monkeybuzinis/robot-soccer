/**
 * RobotPU Soccer — STEP 1: Ball + Goal DETECTION TEST (no navigation/kick).
 * =========================================================================
 * Purpose: isolate and verify the ESP32 camera -> I2C -> micro:bit detection
 * path for BOTH ball and goal before adding localization/planning/kicking
 * (that full pipeline is robotpu-soccer-final.ts). Use this file to answer:
 *   - Is the goal ever reported at all (type/flags/count over serial)?
 *   - At what distance does the ball stop being reported?
 *   - Is one object's detections crowding out the other on the I2C link?
 *
 * Run this as the ONLY .ts file in the MakeCode project (remove the others,
 * they redeclare the same globals -- same caveat as robotpu-soccer-final.ts).
 *
 * Sources reused (see each file's header for the original context):
 *   - robotpu-i2c-cam.ts        -> packet layout constants, i8/i16/u16, flagsText(),
 *                                  setService(), the "print every packet" debug habit.
 *   - robotpu-followball.ts     -> head-tracking pattern that reads the servo's LIVE
 *                                  current target each cycle (robotPuPro.servoTargets())
 *                                  instead of caching a separate yaw/pitch variable --
 *                                  avoids the clamp-state drift bug found in earlier files.
 *   - robotpu-search-soccer.js / robotpu-soccer-mvp.ts -> SEARCH_PATTERN sweep,
 *                                  setSoccerDetection(), handling ball+goal in one
 *                                  trackBall(), VALID/STALE gating.
 *   - robotpu-soccer-final.ts   -> confirmed robotPuPro namespace/casing against the
 *                                  real https://github.com/robotgyms/pxt-robotpu source,
 *                                  and the head servo center=90/range=±45 fix.
 *
 * What this file does, every ~20ms cycle:
 *   - Reads one I2C packet and ALWAYS prints a full debug line (type, flags,
 *     count, score, x/y/z mm, yaw/pitch byte) -- regardless of whether it's a
 *     ball, goal, or anything else. This is the main diagnostic output.
 *   - BALL seen: head-tracks it (yaw/pitch servo), eyes light up, and on the
 *     first frame it's (re)acquired: shows "B" on the LED matrix + speaks "Ball".
 *   - GOAL seen: does NOT move the head (head stays locked onto the ball) --
 *     this deliberately isolates goal detection from head-tracking. On first
 *     (re)acquisition: shows "G" on the LED matrix + speaks "Goal".
 *   - Ball lost for LOST_TIMEOUT_MS: runs a head-scanning search pattern.
 *   - No walking, no kicking -- the body stays in whatever idle/rest mode the
 *     extension defaults to, so you can test detection without the robot
 *     wandering off.
 *   - Reports a "strategy status" on every state change (SEARCHING / WAITING /
 *     READY) mirroring robotpu-soccer-final.ts's "detect both, then plan"
 *     rule: the real pipeline only ever walks once both ball AND goal are
 *     confirmed. This file does no walking, so it just announces (LED icon +
 *     speech + serial line) which of those three states it WOULD be in --
 *     useful for validating that decision logic before it drives movement.
 */

// ---------------------------------------------------------------------------
// I2C protocol / packet layout (from robotpu-i2c-cam.ts)
// ---------------------------------------------------------------------------
const MUX_ADDR = 112 // 0x70 (I2C multiplexer)
const ESP32_ADDR = 66 // 0x42 (camera coprocessor)
const SIZE = 18 // bytes per detection packet

const FACE = 1
const WAKE = 2
const VOICE = 3
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
// Tuning
// ---------------------------------------------------------------------------
const SCAN_WAIT_FRAMES = 25
const LOST_TIMEOUT_MS = 6000
const DEBUG_FLAG = true

// Head servo range: pxt-robotpu's PCB.servoStep() clamps to an ABSOLUTE
// [0,179] degree range, with 90 = looking straight ahead (confirmed against
// the real extension source) -- not a small offset around 0.
const HEAD_YAW_CENTER = 90
const HEAD_PITCH_CENTER = 90
const HEAD_YAW_MIN = HEAD_YAW_CENTER - 45
const HEAD_YAW_MAX = HEAD_YAW_CENTER + 45
const HEAD_PITCH_MIN = HEAD_PITCH_CENTER - 45
const HEAD_PITCH_MAX = HEAD_PITCH_CENTER + 45

function clampL(v: number, lo: number, hi: number): number {
    return Math.min(hi, Math.max(lo, v))
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

function setService(serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}

function setSoccerDetection(enabled: boolean) {
    setService(SERVICE_SOCCER_BALL_DETECTION, enabled)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, enabled)
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
let scanStepIndex = 0
let scanFrameCounter = 0
let search_gain = 1

let yaw = 0
let pitch = 0
let lastBallSeenTime = 0
let lastGoalSeenTime = 0

let ballFound = false
let goalFound = false

// Mirrors robotpu-soccer-final.ts's "detect both, then plan" strategy:
// the real pipeline only ever walks once BOTH ball_valid && goal_valid are
// true. This file drives no motors, so instead we just report which state
// the strategy would be in -- useful for validating that logic in isolation
// before trusting it with movement.
const STATUS_SEARCHING = 0 // neither found (or only the goal, without the ball)
const STATUS_WAITING = 1 // ball found, goal not yet -> would hold position
const STATUS_READY = 2 // both found -> would plan the kick-point approach
let lastStatus = -1

function reportStatusChange(status: number) {
    if (status === lastStatus) return
    lastStatus = status
    if (status === STATUS_READY) {
        serial.writeLine("status: READY (ball+goal both visible -- would plan kick path)")
        basic.showIcon(IconNames.Yes)
        robotPuPro.talk("Ready to kick")
    } else if (status === STATUS_WAITING) {
        serial.writeLine("status: WAITING (ball visible, goal not yet -- holding position)")
        basic.showIcon(IconNames.Square)
        robotPuPro.talk("Waiting for goal")
    } else {
        serial.writeLine("status: SEARCHING (ball not visible)")
        basic.showIcon(IconNames.No)
    }
}

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
// Head-scanning search pattern (ported from robotpu-search-soccer.js /
// robotpu-soccer-mvp.ts). Reads the servo's LIVE current target each call
// (robotpu-followball.ts pattern) instead of a separately cached variable.
// ---------------------------------------------------------------------------
function searchBall(searchPattern: { y: number, p: number }[]) {
    yaw *= 0.5
    pitch *= 0.5

    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = searchPattern[scanStepIndex]
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        const liveYaw = robotPuPro.servoTargets()[4]
        const livePitch = robotPuPro.servoTargets()[5]
        const nextYaw = clampL(liveYaw + targetOffset.y * search_gain, HEAD_YAW_MIN, HEAD_YAW_MAX)
        const nextPitch = clampL(livePitch + targetOffset.p * search_gain, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 1)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 1)
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
// Packet handling: print everything, head-track ball only, flag goal sightings.
// ---------------------------------------------------------------------------
function trackBall(p: Buffer) {
    const currentTime = input.runningTime()

    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    const type = p[0]
    const flags = p[3]
    let count = p[4]
    const score = p[5]
    const x_mm = i16(p, 6)
    const y_mm = i16(p, 8)
    const z_mm = i16(p, 10)
    const yawByte = i8(p[16])
    const pitchByte = i8(p[17])

    // Main diagnostic: see exactly what the camera is reporting, every cycle.
    if (DEBUG_FLAG) {
        serial.writeLine(`type=${type} flags=${flagsText(flags)} count=${count} score=${score} x=${x_mm} y=${y_mm} z=${z_mm} yaw=${yawByte} pitch=${pitchByte}`)
    }

    if (!(flags & VALID)) count = 0

    if (type == SOCCER_BALL) {
        if (count > 0) {
            lastBallSeenTime = currentTime
            search_gain = 1.0
            yaw = yawByte
            pitch = pitchByte
            const staleScale = (flags & STALE) ? 0.3 : 1.0

            robotPuPro.setModeVar(robotPuPro.Mode.API)
            const liveYaw = robotPuPro.servoTargets()[4]
            const livePitch = robotPuPro.servoTargets()[5]
            const nextYaw = clampL(liveYaw + yaw * staleScale * 0.08, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(livePitch + pitch * staleScale * 0.08, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 8)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 8)
            robotPuPro.leftEyeBright(0.01)
            robotPuPro.rightEyeBright(0.01)

            if (!ballFound) {
                ballFound = true
                basic.showString("B")
                robotPuPro.talk("Ball")
            }
        } else if (currentTime - lastBallSeenTime < LOST_TIMEOUT_MS) {
            // Brief follow-through in the last known direction.
            yaw *= 0.7
            pitch *= 0.7
            const liveYaw = robotPuPro.servoTargets()[4]
            const livePitch = robotPuPro.servoTargets()[5]
            const nextYaw = clampL(liveYaw + yaw * 0.2, HEAD_YAW_MIN, HEAD_YAW_MAX)
            const nextPitch = clampL(livePitch + pitch * 0.2, HEAD_PITCH_MIN, HEAD_PITCH_MAX)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, nextYaw, 5)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, nextPitch, 5)
        } else {
            if (ballFound) {
                ballFound = false
                robotPuPro.talk("Where is the ball")
            }
            searchBall(SEARCH_PATTERN)
        }
    } else if (type == SOCCER_GOAL) {
        if (count > 0) {
            lastGoalSeenTime = currentTime
            if (!goalFound) {
                goalFound = true
                basic.showString("G")
                robotPuPro.talk("Goal")
            }
        } else if (goalFound && currentTime - lastGoalSeenTime >= LOST_TIMEOUT_MS) {
            goalFound = false
        }
    }
}

// ---------------------------------------------------------------------------
// Boot sequence
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
    setService(SERVICE_WIFI, true) // keep the camera web UI reachable for cross-checking
    basic.pause(30000)
})

// ---------------------------------------------------------------------------
// Detection-only loop: no localization, no planning, no walking, no kicking.
// ---------------------------------------------------------------------------
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackBall(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }

    const status = !ballFound ? STATUS_SEARCHING : (goalFound ? STATUS_READY : STATUS_WAITING)
    reportStatusChange(status)

    basic.pause(20)
})

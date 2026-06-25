
/**
 * RobotPU soccer: follow ball
 * Turn on soccer ball detection and goal detection.
 * Use I2C to communicate with ESP32, poll detection results.
 * If ball is detected, follow it. if not detected, search for ball.
 */
// Parse Unsigned Char
function i8(v: number) {
    return v >= 128 ? v - 256 : v
}
// Event status to string
function flagsText(f: number) {
    let s = ""
    if (f & VALID) {
        s = "" + s + " valid"
    }
    if (f & STALE) {
        s = "" + s + " stale"
    }
    if (f & CAPTURE) {
        s = "" + s + " capture"
    }
    if (f & WEB) {
        s = "" + s + " web"
    }
    if (f & SLEEP) {
        s = "" + s + " sleep"
    }
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

function searchBall(searchPattern: { y: number, p: number }[]) {
    yaw *= 0.5
    pitch *= 0.5

    if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        const targetOffset = searchPattern[scanStepIndex]
        // if (DEBUG_FLAG) {
        //     serial.writeLine("" + (`yawSearch: ${targetOffset.y * search_gain}`))
        //     serial.writeLine("" + (`pitchSearch: ${targetOffset.p * search_gain}`))
        // }
        robotPuPro.setModeVar(robotPuPro.Mode.API)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, currentYaw + targetOffset.y * search_gain, 1)
        robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, currentPitch + targetOffset.p * search_gain, 1)
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

const MUX_ADDR = 112 // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

const SOCCER_BALL = 4
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

let scanStepIndex = 0
let scanFrameCounter = 0

let currentPitch = 0
let currentYaw = 0

let yaw = 0
let pitch = 0
let lastBallSeenTime = 0
let search_gain = 1
let walkSpeed = 0
let walkTurn = 0
let soccerFound = 0

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

robotPuPro.setChannel(166)
// set servo trim to help robot balancing
robotPuPro.setServoTrim(-5, 0, -5, 0, -9, 0)
// start position tracking from a known (0,0,0) reference
robotPuPro.resetOdom()
radio.onReceivedString(function (receivedString) {
    robotPuPro.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name, value) {
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
function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}
function u16(buf: Buffer, offset: number): number {
    return buf[offset] | (buf[offset + 1] << 8)
}

function trackBall(p: Buffer) {
    const currentTime = input.runningTime()

    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    let type = p[0]
    let flags = p[3]
    let count = p[4]

    // note: may need to check valid flag
    // && !(flags & STALE)
    // it will tell you whether the result is old
    if (type == SOCCER_BALL) {
        if (count > 0 && !(flags & STALE)) {
            lastBallSeenTime = currentTime
            search_gain = 1.0
            let x_mm = i16(p, 6)
            let y_mm = i16(p, 8)
            // let z_mm = i16(p, 10)
            // let w = u16(p, 12)
            // let h = u16(p, 14)
            yaw = i8(p[16])
            pitch = i8(p[17])
            if (DEBUG_FLAG) {
                // serial.writeLine(`head yaw: ${robotPuPro.servoTargets()[4]}`)
                //serial.writeLine(`yawLock ${yaw}`)
                // serial.writeLine(`head pitch: ${robotPuPro.servoTargets()[5]}`)
                //serial.writeLine(`pitchLock: ${pitch}`)
                serial.writeLine(`ball x: ${x_mm}`)
                serial.writeLine(`ball y: ${y_mm}`)
            }
            // move head to look at the ball
            robotPuPro.setModeVar(robotPuPro.Mode.API)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, robotPuPro.servoTargets()[4] + yaw * 0.2, 8)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, robotPuPro.servoTargets()[5] + pitch * 0.2, 8)
            robotPuPro.leftEyeBright(0.01)
            robotPuPro.rightEyeBright(0.01)
            // compute the speed and direction to walk toward the ball 
            // (simple method: forward based on range, turn based on yaw)
            // Note: tune these gains for your field and camera.
            // to do: map y_mm to walk speed, map yaw to turn speed (clamp to -1, 1)
            // stop at 100mm away from the ball
            walkSpeed = Math.max(-3, Math.min(3, (y_mm - 150) * 0.015))
            walkTurn = Math.max(-1, Math.min(1, (walkTurn + yaw * -0.05) * 0.5))
            // cache head pitch/yaw
            currentYaw = robotPuPro.servoTargets()[4]
            currentPitch = robotPuPro.servoTargets()[5]
            if (DEBUG_FLAG) {
                serial.writeLine(`walkSpeed: ${walkSpeed}`)
                serial.writeLine(`walkTurn: ${walkTurn}`)
            }
            if (soccerFound == 0){
                soccerFound = 1
                robotPuPro.talk("Soccer Ball")
            }
        } else if (currentTime - lastBallSeenTime < LOST_TIMEOUT_MS) {
            // follow through with decay for a short moment if the ball is lost from view
            yaw *= 0.7
            pitch *= 0.7
            walkSpeed *= 0.7 // adjust it to tweak follow through
            walkTurn *= 0.9 // decay direction slower
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadYaw, robotPuPro.servoTargets()[4] + yaw * 0.2, 5)
            robotPuPro.servoStep(robotPuPro.ServoJoint.HeadPitch, robotPuPro.servoTargets()[5] + pitch * 0.2, 5)
            // cache head pitch/yaw
            currentYaw = robotPuPro.servoTargets()[4]
            currentPitch = robotPuPro.servoTargets()[5]
            if (DEBUG_FLAG) {
                serial.writeLine(`walkSpeed: ${walkSpeed}`)
                serial.writeLine(`walkTurn: ${walkTurn}`)
            }
        } else {
            // stop the robot when the ball has been lost for a long time
            walkSpeed = 0
            walkTurn = 0
            // lost the ball, search for ball
            searchBall(SEARCH_PATTERN)
            if (soccerFound == 1){
                soccerFound = 0
                robotPuPro.talk("Where is the ball?")
            }
        }
    }
}
basic.showString("I")
// enable TAC I2C channels
pins.i2cWriteNumber(
    MUX_ADDR,
    15,
    NumberFormat.Int8LE,
    false
)
// wait camera boots up
basic.pause(2000)

// this loop is used to handle camera reboot
// by default, all detection services are off when the camera boots up.
// turn on necessary services here.
// safe to run those commands repeatedly, camera handles them well.
basic.forever(function () {
    // turn on image capture
    setService(SERVICE_IMAGE_CAPTURE, true)
    basic.pause(10)
    // turn on soccer ball detection
    setSoccerDetection(true)
    basic.pause(10)
    // turn on wifi for debugging
    if (DEBUG_FLAG) {
        setService(SERVICE_WIFI, true)
    } else {
        setService(SERVICE_WIFI, false)
    }
    basic.pause(30000)
})

// cache the head pitch and yaw angle
currentYaw = robotPuPro.servoTargets()[4]
currentPitch = robotPuPro.servoTargets()[5]

// Soccer ball detection loop
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackBall(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }
    basic.pause(20)
})

// robot action loop
basic.forever(function () {
    // use the computed walk speed and turn to move the robot
    robotPuPro.walk(walkSpeed, walkTurn)
    basic.pause(5)
})

// Pose readout: robotPuPro.walk() already updates its internal odometry on
// every completed step, this just reads that pose out over serial.
basic.forever(function () {
    const loc = robotPuPro.locationArray() // [x_mm, y_mm, theta_deg]
    serial.writeLine(`POSE x_mm=${loc[0]} y_mm=${loc[1]} theta_deg=${loc[2]}`)
    basic.pause(250)
})

basic.forever(function(){
    if (soccerFound == 1) {
        robotPuPro.talk("Kick and go go Goal")
    } else {
        robotPuPro.talk("Searching")
    }
    basic.pause(5000)
})
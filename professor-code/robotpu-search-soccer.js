// Parse Unsigned Char
function i8 (v: number) {
    return v >= 128 ? v - 256 : v
}
// Event status to string
function flagsText (f: number) {
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
function setSoccerDetection (enabled: boolean) {
    setService(SERVICE_SOCCER_BALL_DETECTION, enabled)
    basic.pause(10)
    setService(SERVICE_SOCCER_GOAL_DETECTION, enabled)
}
function setService (serviceId: number, enabled: boolean) {
    pins.i2cWriteBuffer(ESP32_ADDR, Buffer.fromArray([CMD_SERVICE_ENABLE, serviceId, enabled ? 1 : 0]), false)
}
radio.onReceivedString(function (receivedString) {
    robotPu.runStringCommand(receivedString)
})
radio.onReceivedValue(function (name, value) {
    robotPu.runKeyValueCommand(name, value)
})
function searchBall () {
    yaw *= 0.5
pitch *= 0.5
if (scanFrameCounter > 0) {
        scanFrameCounter += -1
        let targetOffset22 = SEARCH_PATTERN[scanStepIndex]
        if (DEBUG_FLAG) {
            serial.writeLine("" + (`yawSearch: ${targetOffset22.y * search_gain}`))
            serial.writeLine("" + (`pitchSearch: ${targetOffset22.p * search_gain}`))
        }
        robotPu.setModeVar(robotPu.Mode.API)
        robotPu.servoStep(robotPu.ServoJoint.HeadYaw, currentYaw + targetOffset22.y * search_gain, 1)
        robotPu.servoStep(robotPu.ServoJoint.HeadPitch, currentPitch + targetOffset22.p * search_gain, 1)
        robotPu.leftEyeBright(0.002)
        robotPu.rightEyeBright(0.002)
    } else {
        scanFrameCounter = SCAN_WAIT_FRAMES
        scanStepIndex += 1
        if (scanStepIndex >= SEARCH_PATTERN.length) {
            scanStepIndex = 0
            search_gain *= 1.1
search_gain = Math.min(4, search_gain)
        }
    }
}
/**
 * variables
 */
let scanStepIndex = 0
let targetOffset22 = 0
let scanFrameCounter = 0
let SCAN_WAIT_FRAMES = 0
let SERVICE_SOCCER_GOAL_DETECTION = 0
let SERVICE_SOCCER_BALL_DETECTION = 0
let SEARCH_PATTERN: {y:number, p:number}[] = []
let DEBUG_FLAG = false
let search_gain = 0
// 记录最后一次看见人脸的系统时间戳（毫秒）
let lastFaceSeenTime = 0
let pitch = 0
let yaw = 0
// Event Types
let IDLE = 0
let currentYaw = 0
let currentPitch = 0
let targetOffset = 0
let s = ""
let targetOffset2 = 0
SEARCH_PATTERN = [
{ y: 15, p: 0 },
// 向右看
    { y: -15, p: 0 },
// 向左看
    { y: -15, p: -10 },
// 向左看，抬头
    { y: 0, p: -10 },
// 回正，抬头
    { y: 15, p: -10 },
// 向右看，抬头
    { y: 15, p: 3 },
// 向右看，微微低头
    { y: 0, p: 3 },
// 回正，低头
    { y: -15, p: 3 },
// 向左看，低头
    { y: -15, p: 0 },
// 向左看，
    { y: 0, p: 0 }
]
robotPu.setChannel(166)
// Address
// 0x70
let MUX_ADDR = 112
// 0x42
let ESP32_ADDR = 66
let SIZE = 18
let FACE = 1
let WAKE = 2
let VOICE = 3
let SOCCER_BALL = 4
let SOCCER_GOAL = 5
let CMD_SERVICE_ENABLE = 8
let SERVICE_WIFI = 1
let SERVICE_IMAGE_CAPTURE = 2
let SERVICE_FACE_DETECTION = 3
SERVICE_SOCCER_BALL_DETECTION = 4
SERVICE_SOCCER_GOAL_DETECTION = 5
const VALID = 1 << 0
const STALE = 1 << 1
const CAPTURE = 1 << 2
const WEB = 1 << 3
const SLEEP = 1 << 4
SCAN_WAIT_FRAMES = 25
search_gain = 1
let LOST_TIMEOUT_MS = 6000
DEBUG_FLAG = true
function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}
function u16(buf: Buffer, offset: number): number {
    return buf[offset] | (buf[offset + 1] << 8)
}
robotPu.setServoTrim(
-5,
0,
-5,
0,
-8,
0
)
function trackBall(p: Buffer) {
    // 
    let currentTime = input.runningTime();

    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    let type = p[0]
    let ver = p[1]
    let seq = p[2]
    let flags = p[3]
    let count = p[4]
    let score = p[5]

    if (type == SOCCER_BALL) {
        if (DEBUG_FLAG) {
            serial.writeLine(`type=${type} flag=${flagsText(flags)} objects=${count} score=${score}`)
        }
        if (count > 0) {
            lastFaceSeenTime = currentTime
            search_gain = 1.0
            let x_mm = i16(p, 6)
            let y_mm = i16(p, 8)
            let z_mm = i16(p, 10)
            let w = u16(p, 12)
            let h = u16(p, 14)
            yaw = i8(p[16])
            pitch = i8(p[17])
            if (DEBUG_FLAG) {
                // serial.writeLine(`head yaw: ${robotPu.ServoTargets()[4]}`)
                serial.writeLine(`yawLock ${yaw}`)
                // serial.writeLine(`head pitch: ${robotPu.ServoTargets()[5]}`)
                serial.writeLine(`pitchLock: ${pitch}`)
            }
            robotPu.setModeVar(robotPu.Mode.API)
            robotPu.servoStep(robotPu.ServoJoint.HeadYaw, robotPu.ServoTargets()[4] + yaw * 0.08, 8)
            robotPu.servoStep(robotPu.ServoJoint.HeadPitch, robotPu.ServoTargets()[5] + pitch * 0.08, 8)
            robotPu.leftEyeBright(0.01)
            robotPu.rightEyeBright(0.01)
        } else if (currentTime - lastFaceSeenTime < LOST_TIMEOUT_MS) {
            // lock on face 
            yaw *= 0.7
            pitch *= 0.7
            robotPu.servoStep(robotPu.ServoJoint.HeadYaw, robotPu.ServoTargets()[4] + yaw * 0.2, 5)
            robotPu.servoStep(robotPu.ServoJoint.HeadPitch, robotPu.ServoTargets()[5] + pitch * 0.2, 5)
            // 3. 读取当前头部的绝对目标角度
            currentYaw = robotPu.ServoTargets()[4]
            currentPitch = robotPu.ServoTargets()[5]
        } else {
            // lost the face, search for face
            searchBall()
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
basic.pause(3000)
currentYaw = robotPu.ServoTargets()[4]
currentPitch = robotPu.ServoTargets()[5]
basic.pause(10)
setService(SERVICE_IMAGE_CAPTURE, true)
basic.pause(10)
setService(SERVICE_FACE_DETECTION, false)
basic.pause(10)
setSoccerDetection(true)
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)
    if (packet.length == SIZE) {
        trackBall(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }
    // get robot head pitch yaw angles, 
    // head yaw: 0 if neutal, >0 yaw left, <0 yaw right
    // head pitch: 0 if neutal, > 0 pitch up, <0 pitch down
    let head_yaw = robotPu.ServoTargets()[4] 
    let head_pitch = robotPu.ServoTargets()[5]

    // to do: compute soccer ball location in global (x,y,z)
    // camera is put at (0, 35, 160) millimeters at robot frame.
    basic.pause(20)
})
basic.forever(function () {
    if (DEBUG_FLAG) {
        setService(SERVICE_WIFI, true)
    } else {
        setService(SERVICE_WIFI, false)
    }
    basic.pause(30000)
})

if (input.buttonIsPressed(Button.A)){
    setService(SERVICE_WIFI, true)
}

if (input.buttonIsPressed(Button.B)) {
    setService(SERVICE_WIFI, false)
}

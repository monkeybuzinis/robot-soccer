// Address
const MUX_ADDR = 112  // 0x70
const ESP32_ADDR = 66 // 0x42
const SIZE = 18

// Event Types
const IDLE = 0x00
const FACE = 0x01
const WAKE = 0x02
const VOICE = 0x03
const SOCCER_BALL = 0x04
const SOCCER_GOAL = 0x05

// Event status
const VALID = 1 << 0
const STALE = 1 << 1
const CAPTURE = 1 << 2
const WEB = 1 << 3
const SLEEP = 1 << 4

/**
 * Parse 16-byte package
 */
function i16(buf: Buffer, offset: number): number {
    let v = buf[offset] | (buf[offset + 1] << 8)
    return v >= 32768 ? v - 65536 : v
}

/**
 * Parse U16
 */
function u16(buf: Buffer, offset: number): number {
    return buf[offset] | (buf[offset + 1] << 8)
}

/**
 * Parse Unsigned Char
 */
function i8(v: number): number {
    return v >= 128 ? v - 256 : v
}

/**
 * Event status to string
 */
function flagsText(f: number): string {
    let s = ""
    if (f & VALID) s += " valid"
    if (f & STALE) s += " stale"
    if (f & CAPTURE) s += " capture"
    if (f & WEB) s += " web"
    if (f & SLEEP) s += " sleep"
    return s.length > 0 ? s.trim() : "none"
}

/**
 * print packages
 */
function printPacket(p: Buffer) {
    if (p.length != SIZE) {
        serial.writeLine("bad length: " + p.length)
        return
    }

    let type = p[0]
    let ver = p[1]
    let seq = p[2]
    let flags = p[3]

    if (type == FACE || type == SOCCER_BALL || type == SOCCER_GOAL) {
        let count = p[4]
        let score = p[5]
        let x_mm = i16(p, 6)
        let y_mm = i16(p, 8)
        let z_mm = i16(p, 10)
        let w = u16(p, 12)
        let h = u16(p, 14)
        let yaw = i8(p[16])
        let pitch = i8(p[17])
        if (!(flags & STALE)) {
            serial.writeLine(`type=${type} ver=${ver} seq=${seq} flags=${flagsText(flags)} objects=${count} score=${score} x_mm=${x_mm} y_mm=${y_mm} z_mm=${z_mm} box=${w}x${h} yaw=${yaw} pitch=${pitch}`)
        }
    }
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

basic.showString("I")

// Open all 4 channels in TCA9546A
pins.i2cWriteNumber(MUX_ADDR, 0x0F, NumberFormat.Int8LE, false)

// turn on detections
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

// parse i2c packages
basic.forever(function () {
    let packet = pins.i2cReadBuffer(ESP32_ADDR, SIZE, false)

    if (packet.length == SIZE) {
        printPacket(packet)
    } else {
        serial.writeLine("i2c read error")
        basic.showIcon(IconNames.No)
    }

    //serial.writeLine("---")
    basic.pause(20)
})
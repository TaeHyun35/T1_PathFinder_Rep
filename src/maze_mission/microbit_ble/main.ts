// AiHand over Bluetooth (micro:bit MakeCode).
// Same servo control as the team's USB program (microbit-serial-range.hex):
//   micro:bit -> Hiwonder servo board on P12(TX)/P8(RX) 115200, frame 55 55 len 03 n tL tH [id pL pH]..
// Commands come over Bluetooth UART, one per line. BLE UART carries only 20 bytes per
// packet, so commands are short:
//   s<i>,<deg>   set target of servo i (1 thumb, 2 index, 3 middle, 4 ring, 5 little, 6 wrist), 0..180
//   m<ms>        move all servos to their targets in <ms> (0..1000)        -> "OK"
//   o<ms>        open hand (OPEN pose) in <ms>                             -> "OK"
//   ?            current targets                                           -> "T a,b,c,d,e,f"
// Anything else -> "ERR ..."
// Project settings: Bluetooth extension, "No Pairing Required".

const OPEN = [50, 130, 130, 130, 130, 90]
let target: number[] = []
for (let k = 0; k < 6; k++) {
    target.push(OPEN[k])
}

serial.redirect(SerialPin.P12, SerialPin.P8, BaudRate.BaudRate115200)
bluetooth.startUartService()
basic.showIcon(IconNames.Yes)

bluetooth.onBluetoothConnected(function () {
    basic.showIcon(IconNames.Happy)
})
bluetooth.onBluetoothDisconnected(function () {
    basic.showIcon(IconNames.Yes)
})

function sendToBoard(ms: number) {
    let buf = pins.createBuffer(5 + 3 * 6 + 2)
    buf[0] = 0x55
    buf[1] = 0x55
    buf[2] = 3 * 6 + 5
    buf[3] = 0x03
    buf[4] = 6
    buf[5] = ms & 0xff
    buf[6] = (ms >> 8) & 0xff
    for (let i = 0; i < 6; i++) {
        let pulse = Math.round(500 + target[i] * 2000 / 180)
        buf[7 + i * 3] = i + 1
        buf[8 + i * 3] = pulse & 0xff
        buf[9 + i * 3] = (pulse >> 8) & 0xff
    }
    serial.writeBuffer(buf)
}

function clampMs(s: string): number {
    let ms = parseInt(s)
    if (isNaN(ms)) {
        ms = 300
    }
    return Math.max(0, Math.min(1000, ms))
}

bluetooth.onUartDataReceived(serial.delimiters(Delimiters.NewLine), function () {
    let raw = bluetooth.uartReadUntil(serial.delimiters(Delimiters.NewLine))
    // keep only command letters, digits and commas (drops \r and stray bytes)
    let line = ""
    for (let i = 0; i < raw.length; i++) {
        let ch = raw.charAt(i)
        if (ch == "," || ch == "?" || (ch >= "0" && ch <= "9") || ch == "s" || ch == "m" || ch == "o") {
            line = line + ch
        }
    }
    if (line.length == 0) {
        return
    }
    let cmd = line.charAt(0)
    let rest = line.substr(1, line.length - 1)
    if (cmd == "s") {
        let p = rest.split(",")
        let idx = parseInt(p[0])
        let deg = parseInt(p[1])
        if (p.length != 2 || isNaN(idx) || isNaN(deg) || idx < 1 || idx > 6) {
            bluetooth.uartWriteLine("ERR " + line)
            return
        }
        target[idx - 1] = Math.max(0, Math.min(180, deg))
    } else if (cmd == "m") {
        sendToBoard(clampMs(rest))
        bluetooth.uartWriteLine("OK")
    } else if (cmd == "o") {
        for (let k = 0; k < 6; k++) {
            target[k] = OPEN[k]
        }
        sendToBoard(clampMs(rest))
        bluetooth.uartWriteLine("OK")
    } else if (cmd == "?") {
        bluetooth.uartWriteLine("T " + target.join(","))
    } else {
        bluetooth.uartWriteLine("ERR " + line)
    }
})

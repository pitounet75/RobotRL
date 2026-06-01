#!/usr/bin/env python3
"""Map KiCad U5 net labels to MCU pins via schematic coordinates."""
import re
from pathlib import Path

SCH = Path(__file__).resolve().parents[1] / "STM32GenericBoard" / "STM32GenericBoard.kicad_sch"
IOC = Path(__file__).resolve().parents[1] / "STM32H743LFQP100Pinout" / "STM32H743LFQP100Pinout.ioc"

U5_AT = (46.99, 107.95, 0)  # x, y, rot_deg
TOL = 0.02

LABEL_TO_PORT = {
    "PH0_OSCIN": "PH0",
    "PH1_OSCOUT": "PH1",
    "SWIO": "PA13",
    "SWCLK": "PA14",
    "SWO": "PB3",
    "UART4_TX": "PA0",
    "UART4_RX": "PD0",
    "UART5_TX": "PC12",
    "UART5_RX": "PD2",
    "SPI1_SCK": "PA5",
    "SPI1_MOSI": "PB5",
    "SPI1_MISO": "PB4",
    "SPI1_MSIO": "PB4",
    "SPI2_SCK": "PB10",
    "SPI2_MOSI": "PC1",
    "SPI2_MISO": "PC2_C",
    "FDCAN2_RX": "PB12",
    "FDCAN2_TX": "PB13",
    "FDCAN1_RX": "PB8",
    "FDCAN1_TX": "PD1",
    "I2C1_SCL": "PB6",
    "I2C1_SDA": "PB7",
    "USB_OTG_DM": "PA11",
    "USB_OTG_DP": "PA12",
    "TIM2_CH1": "PA15",
    "TIM2_CH2": "PA1",
    "TIM4_CH1": "PD12",
    "TIM4_CH2": "PD13",
    "TIM1_CH1": "PE9",
    "TIM1_CH2": "PE11",
    "TIM1_CH3": "PE13",
    "TIM1_CH4": "PE14",
    "PB0": "PB0",
    "PB1": "PB1",
    "PB2": "PB2",
    "PC5": "PC5",
    "PE7": "PE7",
    "PE8": "PE8",
    "PE4_SPI1_CS": "PE4",
    "PE5_SPI1_CS": "PE5",
    "PE6_SPI2_CS": "PE6",
    "SPI3_SCK": "PC10",
    "SPI3_MOSI": "PC11",
    "SPI3_MISO": "PC12",
}


def parse_ioc():
    pins = {}
    for line in IOC.read_text(encoding="utf-8", errors="replace").splitlines():
        if ".Signal=" not in line or not line[0].isalpha():
            continue
        port_part, sig = line.split(".Signal=", 1)
        raw = port_part.strip().replace("\\\\", "")
        if "PH0" in raw:
            port = "PH0"
        elif "PH1" in raw:
            port = "PH1"
        elif raw.startswith("PA13"):
            port = "PA13"
        elif raw.startswith("PA14"):
            port = "PA14"
        elif raw.startswith("PA15"):
            port = "PA15"
        elif raw.startswith("PB3"):
            port = "PB3"
        elif raw.startswith("PB4"):
            port = "PB4"
        else:
            port = raw.split("(")[0].split("-")[0]
        pins[port] = sig.strip()
    return pins


def parse_symbol_pins(text):
    """Return list of (pin_number, gpio_name, rel_x, rel_y, angle)."""
    pins = []
    start = text.find('(symbol "STM32H743VGTx_1_1"')
    if start < 0:
        return pins
    body = text[start : start + 250000]
    for m in re.finditer(
        r'\(pin \w+ line\s+\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s+\(length [\d.]+\)\s+\(name "([^"]+)"',
        body,
    ):
        rx, ry, ang, name = m.groups()
        num_m = re.search(
            r'\(number "(\d+)"',
            body[m.end() : m.end() + 120],
        )
        if not num_m:
            continue
        if name.startswith("~") or name in {
            "VDD",
            "VSS",
            "VSSA",
            "VDDA",
            "VREF+",
            "VREF-",
            "VBAT",
            "BOOT0",
        }:
            continue
        pins.append((num_m.group(1), name, float(rx), float(ry), int(ang)))
    return pins


def pin_anchor_abs(rx, ry, angle):
    sx, sy, rot = U5_AT
    if rot == 0:
        return sx + rx, sy + ry
  # extend if rotation != 0
    return sx + rx, sy + ry


def main():
    text = SCH.read_text(encoding="utf-8", errors="replace")
    sym_pins = parse_symbol_pins(text)
    if not sym_pins:
        print("ERROR: no symbol pins parsed")
        return
    ioc = parse_ioc()

    anchors = []
    for num, gpio, rx, ry, ang in sym_pins:
        ax, ay = pin_anchor_abs(rx, ry, ang)
        bus_x = round(ax, 2)
        anchors.append((gpio, num, ax, ay, bus_x))

    labels = []
    for m in re.finditer(r'\(label "([^"]+)"\s+\(at ([-\d.]+) ([-\d.]+)', text):
        labels.append((m.group(1), float(m.group(2)), float(m.group(3))))

    # Labels on vertical buses beside U5
    mcu_labels = [
        (name, x, y)
        for name, x, y in labels
        if abs(x - 69.85) < 0.1 or abs(x - 24.13) < 0.1
    ]

    def gpio_at(y, bus_x):
        best = None
        best_d = 1e9
        for gpio, num, ax, ay, bx in anchors:
            if abs(bx - bus_x) > 0.1:
                continue
            d = abs(ay - y)
            if d < best_d:
                best_d = d
                best = gpio
        return best if best_d < 1.5 else None

    print("=== Label on bus -> actual GPIO pin (coordinate match) ===\n")
    mismatches = []
    ok = []
    extra = []

    for name, x, y in sorted(mcu_labels, key=lambda t: t[2]):
        actual = gpio_at(y, x)
        expected = LABEL_TO_PORT.get(name)
        if expected is None:
            if name not in ("BOOT0", "VSSA", "VREF"):
                extra.append((name, x, y, actual))
            continue
        if actual is None:
            mismatches.append((name, expected, "?", y, "no pin at this Y"))
            continue
        ioc_sig = ioc.get(actual, "(not in IOC)")
        if actual == expected:
            ok.append((name, actual, ioc_sig, y))
        else:
            mismatches.append((name, expected, actual, y, ioc_sig))

    for row in ok:
        print(f"  OK  {row[0]:18} @ y={row[3]:6.2f} -> {row[1]}  ({row[2]})")

    print("\n=== MISMATCH (label name vs pin at that wire Y) ===\n")
    for row in mismatches:
        print(
            f"  ERR {row[0]:18} @ y={row[3]:6.2f}: label implies {row[1]}, wire is {row[2]}  ({row[4]})"
        )

    if extra:
        print("\n=== Other bus labels ===\n")
        for row in extra:
            print(f"  ?   {row[0]:18} @ y={row[2]:6.2f} -> {row[3]}")

    # IOC pins without labels on buses
    labeled_actual = {gpio_at(y, x) for name, x, y in mcu_labels if LABEL_TO_PORT.get(name)}
    print("\n=== IOC pins with no matching bus label ===\n")
    for p, s in sorted(ioc.items()):
        if p not in labeled_actual and p not in {"PB0", "PB1", "PB2", "PC5", "PE7", "PE8"}:
            print(f"  {p}: {s}")


if __name__ == "__main__":
    main()

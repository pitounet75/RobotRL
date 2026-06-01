#!/usr/bin/env python3
"""Union-find net connectivity for U5 labels vs GPIO pins."""
import re
from pathlib import Path
from collections import defaultdict

SCH = Path(__file__).resolve().parents[1] / "STM32GenericBoard" / "STM32GenericBoard.kicad_sch"
IOC = Path(__file__).resolve().parents[1] / "STM32H743LFQP100Pinout" / "STM32H743LFQP100Pinout.ioc"
U5 = (46.99, 107.95, 0)
EPS = 0.05

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


class UF:
    def __init__(self):
        self.p = {}

    def find(self, x):
        if x not in self.p:
            self.p[x] = x
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def pt_key(x, y):
    return (round(x, 2), round(y, 2))


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
    pins = []
    start = text.find('(symbol "STM32H743VGTx_1_1"')
    body = text[start : start + 250000]
    for m in re.finditer(
        r'\(pin \w+ line\s+\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s+\(length ([\d.]+)\)\s+\(name "([^"]+)"',
        body,
    ):
        rx, ry, ang, length, name = m.groups()
        if name.startswith("~") or name in {"VDD", "VSS", "VSSA", "VDDA", "VREF+", "VREF-", "VBAT"}:
            continue
        pins.append((name, float(rx), float(ry), int(ang), float(length)))
    return pins


def pin_wire_points(gpio, rx, ry, ang, length):
    sx, sy, _ = U5
    ax, ay = sx + rx, sy + ry
  # KiCad: line from (at) in direction angle, length L
    import math

    rad = math.radians(ang)
    ex = ax + length * math.cos(rad)
    ey = ay + length * math.sin(rad)
    return [pt_key(ax, ay), pt_key(ex, ey)]


def main():
    text = SCH.read_text(encoding="utf-8", errors="replace")
    ioc = parse_ioc()
    uf = UF()
    points = []

    for m in re.finditer(
        r'\(wire\s+\(pts\s+\(xy ([-\d.]+) ([-\d.]+)\) \(xy ([-\d.]+) ([-\d.]+)\)',
        text,
    ):
        x1, y1, x2, y2 = map(float, m.groups())
        p1, p2 = pt_key(x1, y1), pt_key(x2, y2)
        points.extend([p1, p2])
        uf.union(p1, p2)

    for m in re.finditer(r'\(label "([^"]+)"\s+\(at ([-\d.]+) ([-\d.]+)', text):
        name, x, y = m.group(1), float(m.group(2)), float(m.group(3))
        if name in ("BOOT0", "VSSA", "VREF"):
            continue
        p = pt_key(x, y)
        points.append(p)
        label_net = uf.find(p)
        # store on point via dict later

    labels = []
    for m in re.finditer(r'\(label "([^"]+)"\s+\(at ([-\d.]+) ([-\d.]+)', text):
        name, x, y = m.group(1), float(m.group(2)), float(m.group(3))
        if name in ("BOOT0", "VSSA", "VREF"):
            continue
        labels.append((name, pt_key(x, y)))

    gpio_points = {}
    for gpio, rx, ry, ang, length in parse_symbol_pins(text):
        for p in pin_wire_points(gpio, rx, ry, ang, length):
            gpio_points.setdefault(p, set()).add(gpio)
            # also union pin points into graph
            points.append(p)

    # Re-run union including pin-only points connected if on same coord as wire
    uf2 = UF()
    all_pts = set()
    for m in re.finditer(
        r'\(wire\s+\(pts\s+\(xy ([-\d.]+) ([-\d.]+)\) \(xy ([-\d.]+) ([-\d.]+)\)',
        text,
    ):
        x1, y1, x2, y2 = map(float, m.groups())
        p1, p2 = pt_key(x1, y1), pt_key(x2, y2)
        all_pts.update([p1, p2])
        uf2.union(p1, p2)

    for gpio, rx, ry, ang, length in parse_symbol_pins(text):
        for p in pin_wire_points(gpio, rx, ry, ang, length):
            all_pts.add(p)

    for _, lp in labels:
        all_pts.add(lp)

    # Junction: if pin point within EPS of wire point, union
    pin_pts = []
    for gpio, rx, ry, ang, length in parse_symbol_pins(text):
        for p in pin_wire_points(gpio, rx, ry, ang, length):
            pin_pts.append((gpio, p))

    wire_pts = list(all_pts)
    for gpio, pp in pin_pts:
        for wp in wire_pts:
            if abs(pp[0] - wp[0]) <= EPS and abs(pp[1] - wp[1]) <= EPS:
                uf2.union(pp, wp)

    def net_gpio_at(label_pt):
        root = uf2.find(label_pt)
        members = [p for p in all_pts if uf2.find(p) == root]
        gpios = set()
        for gpio, pp in pin_pts:
            if uf2.find(pp) == root:
                gpios.add(gpio)
        return gpios, members

    print("=== Connectivity check (label text vs GPIO on same net) ===\n")
    ok = []
    bad = []
    for name, lp in sorted(labels, key=lambda t: t[1][1]):
        expected = LABEL_TO_PORT.get(name)
        if expected is None:
            continue
        gpios, _ = net_gpio_at(lp)
        if not gpios:
            bad.append((name, expected, "?", "label not wired to U5 pin"))
            continue
        actual = sorted(gpios)[0] if len(gpios) == 1 else "/".join(sorted(gpios))
        if len(gpios) > 1:
            bad.append((name, expected, actual, "multiple GPIO on net"))
        elif expected in gpios or (name == "SPI1_MSIO" and "PB4" in gpios):
            ok.append((name, actual, ioc.get(actual, "")))
        else:
            bad.append((name, expected, actual, ioc.get(actual, "")))

    for row in ok:
        print(f"  OK   {row[0]:18} -> {row[1]}  ({row[2]})")
    print()
    for row in bad:
        print(f"  ERR  {row[0]:18} expects {row[1]}, net has {row[2]}  [{row[3]}]")

    print(f"\nSummary: {len(ok)} OK, {len(bad)} errors, {len(labels)} labels total")


if __name__ == "__main__":
    main()

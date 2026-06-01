#!/usr/bin/env python3
"""Compare U5 labels in KiCad against the STM32CubeMX IOC pinout."""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCH = ROOT / "STM32GenericBoard" / "STM32GenericBoard.kicad_sch"
IOC = ROOT / "STM32H743LFQP100Pinout" / "STM32H743LFQP100Pinout.ioc"
EPS = 0.05

SKIP_LABELS = {"BOOT0", "VSSA", "VREF", "+3V3", "GND", "VDD", "VSS"}
GPIO_RE = re.compile(r"^(P[A-Z][0-9]+(?:_C)?|PH[0-9]+)$")


def normalize_pin(raw):
    raw = raw.strip().replace("\\\\", "")
    if "PH0" in raw:
        return "PH0"
    if "PH1" in raw:
        return "PH1"
    for debug_pin in ("PA13", "PA14", "PA15", "PB3", "PB4"):
        if raw.startswith(debug_pin):
            return debug_pin
    return raw.split("(")[0].split("-")[0]


def signal_to_label(pin, signal, gpio_labels):
    if pin in gpio_labels:
        return gpio_labels[pin]
    if signal == "GPIO_Input":
        return pin
    if signal == "ADCx_INP3":
        return "ADC1_INP3"
    if signal == "DEBUG_JTMS-SWDIO":
        return "SWIO"
    if signal == "DEBUG_JTCK-SWCLK":
        return "SWCLK"
    if signal == "DEBUG_JTDO-SWO":
        return "SWO"
    if signal == "RCC_OSC_IN":
        return "PH0_OSCIN"
    if signal == "RCC_OSC_OUT":
        return "PH1_OSCOUT"
    if signal.startswith("S_TIM"):
        return signal[2:].replace("_ETR", "")
    if signal.startswith("USB_OTG_FS_"):
        return signal.replace("USB_OTG_FS_", "USB_OTG_")
    return signal


def parse_ioc_labels():
    signals = {}
    gpio_labels = {}
    for line in IOC.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or not line[0].isalpha():
            continue
        if ".GPIO_Label=" in line:
            raw, label = line.split(".GPIO_Label=", 1)
            gpio_labels[normalize_pin(raw)] = label.strip()
        elif ".Signal=" in line:
            raw, signal = line.split(".Signal=", 1)
            pin = normalize_pin(raw)
            if not pin.startswith(("P", "VP_")):
                continue
            signals[pin] = signal.strip()
    label_to_pin = {}
    pin_to_label = {}
    for pin, signal in signals.items():
        if pin.startswith("VP_"):
            continue
        label = signal_to_label(pin, signal, gpio_labels)
        label_to_pin[label] = pin
        pin_to_label[pin] = label
    return label_to_pin, pin_to_label


def parse_symbol_pins(text):
    start = text.find('(symbol "STM32H743VGTx_1_1"')
    body = text[start : start + 250000]
    pins = []
    for m in re.finditer(
        r'\(pin \w+ line\s+\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s+\(length ([\d.]+)\)\s+\(name "([^"]+)"',
        body,
    ):
        name = m.group(5)
        if not GPIO_RE.match(name):
            continue
        pins.append(
            (
                name,
                float(m.group(1)),
                float(m.group(2)),
                int(m.group(3)),
                float(m.group(4)),
            )
        )
    return pins


def pt(x, y):
    return (round(x, 2), round(y, 2))


class UF:
    def __init__(self):
        self.parent = {}

    def find(self, item):
        if item not in self.parent:
            self.parent[item] = item
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, a, b):
        ra = self.find(a)
        rb = self.find(b)
        if ra != rb:
            self.parent[rb] = ra


def parse_u5_origin(text):
    m = re.search(
        r'\(symbol\s+\(lib_id "MCU_ST_STM32H7:STM32H743VGTx"\)\s+\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)',
        text,
        re.S,
    )
    if not m:
        raise RuntimeError("Unable to find STM32H743VGTx instance")
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def build_pin_points(sym_pins, origin):
    sx, sy, rot = origin
    if abs(rot) > 0.01:
        raise RuntimeError("U5 rotation is not handled")
    points = {}
    for gpio, rx, ry, _ang, _length in sym_pins:
        points[pt(sx + rx, sy - ry)] = gpio
    return points


def parse_labels(text):
    labels = []
    for m in re.finditer(r'\((?:global_)?label "([^"]+)"\s+.*?\(at ([-\d.]+) ([-\d.]+)', text, re.S):
        name = m.group(1)
        if name in SKIP_LABELS:
            continue
        labels.append((name, pt(float(m.group(2)), float(m.group(3)))))
    return labels


def build_wire_nets(text, pin_points):
    uf = UF()
    for m in re.finditer(
        r"\(wire\s+\(pts\s+\(xy ([-\d.]+) ([-\d.]+)\) \(xy ([-\d.]+) ([-\d.]+)\)",
        text,
    ):
        p1 = pt(float(m.group(1)), float(m.group(2)))
        p2 = pt(float(m.group(3)), float(m.group(4)))
        uf.union(p1, p2)
    for pin_point in pin_points:
        uf.find(pin_point)
    return uf


def find_pin_for_label(label_point, pin_points, wire_nets):
    for pin_point, pin in pin_points.items():
        if abs(label_point[0] - pin_point[0]) <= EPS and abs(label_point[1] - pin_point[1]) <= EPS:
            return pin
    label_net = wire_nets.find(label_point)
    net_pins = sorted(pin for pin_point, pin in pin_points.items() if wire_nets.find(pin_point) == label_net)
    if len(net_pins) == 1:
        return net_pins[0]
    if net_pins:
        return "/".join(net_pins)
    return None


def main():
    text = SCH.read_text(encoding="utf-8", errors="replace")
    label_to_ioc_pin, pin_to_ioc_label = parse_ioc_labels()
    origin = parse_u5_origin(text)
    sym_pins = parse_symbol_pins(text)
    pin_points = build_pin_points(sym_pins, origin)
    wire_nets = build_wire_nets(text, pin_points)

    rows = []
    labels_on_u5 = {}
    for name, label_point in parse_labels(text):
        kicad_pin = find_pin_for_label(label_point, pin_points, wire_nets)
        if not kicad_pin:
            continue
        labels_on_u5[name] = kicad_pin
        ioc_pin = label_to_ioc_pin.get(name)
        if not ioc_pin:
            # Detect likely GPIO-label typo like PE4_SPI1_CS vs IOC PE4_SPI3_CS.
            prefix = re.match(r"^(P[A-Z][0-9]+(?:_C)?)_", name)
            if prefix and prefix.group(1) in pin_to_ioc_label:
                ioc_pin = prefix.group(1)
        rows.append((name, ioc_pin or "(absent IOC)", kicad_pin))

    rows.sort(key=lambda r: (r[2], r[0]))

    out = sys.stdout
    out.write("| label | pin IOC | pin KiCad |\n")
    out.write("|-------|---------|----------|\n")
    for name, ioc_pin, kicad_pin in rows:
        out.write(f"| {name} | {ioc_pin} | {kicad_pin} |\n")

    expected_missing = []
    for pin, expected_label in sorted(pin_to_ioc_label.items()):
        actual_labels = [name for name, kicad_pin in labels_on_u5.items() if kicad_pin == pin]
        if expected_label not in actual_labels:
            expected_missing.append((expected_label, pin, ", ".join(actual_labels) or "-"))

    if expected_missing:
        out.write("\n### Labels IOC absents ou differents sur la pin attendue\n\n")
        out.write("| label IOC attendu | pin IOC | label KiCad trouve |\n")
        out.write("|-------------------|---------|--------------------|\n")
        for label, pin, found in expected_missing:
            out.write(f"| {label} | {pin} | {found} |\n")


if __name__ == "__main__":
    main()

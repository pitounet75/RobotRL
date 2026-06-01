#!/usr/bin/env python3
"""Print U5 pin bus Y coordinates for label placement."""
import re
from pathlib import Path

SCH = Path(__file__).resolve().parents[1] / "STM32GenericBoard" / "STM32GenericBoard.kicad_sch"
U5 = (46.99, 107.95)
IOC_PORTS = {
    "PH0", "PH1", "PA0", "PA1", "PA5", "PA6", "PA10", "PA11", "PA12", "PA13", "PA14", "PA15",
    "PB0", "PB1", "PB2", "PB3", "PB4", "PB5", "PB6", "PB7", "PB8", "PB10", "PB12", "PB13",
    "PC1", "PC2_C", "PC5", "PC12", "PD0", "PD1", "PD2", "PD12", "PD13",
    "PE7", "PE8", "PE9", "PE11", "PE13", "PE14",
}

text = SCH.read_text(encoding="utf-8", errors="replace")
start = text.find('(symbol "STM32H743VGTx_1_1"')
body = text[start : start + 250000]
sx, sy = U5[0], U5[1]
rows = []
for m in re.finditer(
    r'\(pin \w+ line\s+\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s+\(length [\d.]+\)\s+\(name "([^"]+)"',
    body,
):
    rx, ry, ang, name = float(m.group(1)), float(m.group(2)), int(m.group(3)), m.group(4)
    if name not in IOC_PORTS:
        continue
    y = round(sy + ry, 2)
    bus = "left 24.13" if rx < 0 else "right 69.85"
    rows.append((y, name, bus))

LABEL_TO_PORT = {
    "PH0_OSCIN": "PH0", "PH1_OSCOUT": "PH1", "SWIO": "PA13", "SWCLK": "PA14", "SWO": "PB3",
    "UART4_TX": "PA0", "UART4_RX": "PD0", "UART5_TX": "PC12", "UART5_RX": "PD2",
    "SPI1_SCK": "PA5", "SPI1_MOSI": "PB5", "SPI1_MISO": "PB4", "SPI1_MSIO": "PB4",
    "SPI2_SCK": "PB10", "SPI2_MOSI": "PC1", "SPI2_MISO": "PC2_C",
    "FDCAN2_RX": "PB12", "FDCAN2_TX": "PB13", "FDCAN1_RX": "PB8", "FDCAN1_TX": "PD1",
    "I2C1_SCL": "PB6", "I2C1_SDA": "PB7", "USB_OTG_DM": "PA11", "USB_OTG_DP": "PA12",
    "TIM2_CH1": "PA15", "TIM2_CH2": "PA1", "TIM4_CH1": "PD12", "TIM4_CH2": "PD13",
    "TIM1_CH1": "PE9", "TIM1_CH2": "PE11", "TIM1_CH3": "PE13", "TIM1_CH4": "PE14",
    "PB0": "PB0", "PB1": "PB1", "PB2": "PB2", "PC5": "PC5", "PE7": "PE7", "PE8": "PE8",
}

y_to_gpio = {}
for y, name, bus in rows:
    y_to_gpio[round(y, 2)] = name

print("=== Bus Y -> GPIO (for label placement) ===\n")
for y, name, bus in sorted(rows):
    print(f"  y={y:7.2f}  {name:6}  ({bus})")

print("\n=== Labels on right bus x=69.85 (text vs pin at same Y) ===\n")
labels = re.findall(r'\(label "([^"]+)"\s+\(at 69\.85 ([-\d.]+)', text)
for name, ys in sorted(labels, key=lambda t: float(t[1])):
    y = round(float(ys), 2)
    actual = y_to_gpio.get(y, "?")
    expected = LABEL_TO_PORT.get(name, "?")
    mark = "OK" if actual == expected else "ERR"
    print(f"  {mark}  y={y:7.2f}  label={name:18}  pin={actual:6}  (IOC expects {expected})")

print("\n=== Labels on left bus x=24.13 ===\n")
labels_l = re.findall(r'\(label "([^"]+)"\s+\(at 24\.13 ([-\d.]+)', text)
for name, ys in sorted(labels_l, key=lambda t: float(t[1])):
    y = round(float(ys), 2)
    actual = y_to_gpio.get(y, "?")
    expected = LABEL_TO_PORT.get(name, "?")
    mark = "OK" if actual == expected else "ERR"
    print(f"  {mark}  y={y:7.2f}  label={name:18}  pin={actual:6}  (IOC expects {expected})")

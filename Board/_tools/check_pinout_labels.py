#!/usr/bin/env python3
"""Compare KiCad schematic net labels on U5 vs STM32H743LFQP100Pinout.ioc."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IOC = ROOT / "STM32H743LFQP100Pinout" / "STM32H743LFQP100Pinout.ioc"
SCH = ROOT / "STM32GenericBoard" / "STM32GenericBoard.kicad_sch"


def parse_ioc(path: Path) -> dict[str, str]:
    text = path.read_text(encoding="utf-8", errors="replace")
    pins: dict[str, str] = {}
    for line in text.splitlines():
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
    "SPI1_MSIO": "PB4",  # typo on schematic
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


def expected_port(label: str):
    if label in LABEL_TO_PORT:
        return LABEL_TO_PORT[label]
    m = re.match(r"^(P[A-Z][0-9]+(?:_C)?)$", label)
    if m:
        return m.group(1)
    return None


def main() -> None:
    ioc_pins = parse_ioc(IOC)
    sch = SCH.read_text(encoding="utf-8", errors="replace")
    labels = sorted(set(re.findall(r'\(label "([^"]+)"', sch)))

    print("=== IOC configured pins ===")
    for p in sorted(ioc_pins):
        print(f"  {p}: {ioc_pins[p]}")

    ok: list[tuple[str, str, str]] = []
    wrong: list[tuple[str, str, str]] = []
    unmapped: list[str] = []
    skip_power = {"VSSA", "VREF", "BOOT0", "+3V3", "GND"}

    for label in labels:
        if label in skip_power:
            continue
        port = expected_port(label)
        if port is None:
            unmapped.append(label)
            continue
        if port not in ioc_pins:
            wrong.append((label, port, "GPIO not configured in pinout IOC"))
        else:
            ok.append((label, port, ioc_pins[port]))

    print(f"\n=== OK ({len(ok)}) ===")
    for row in ok:
        print(f"  {row[0]} -> {row[1]} ({row[2]})")

    typos = [l for l in labels if "MSIO" in l]
    spi3 = [l for l in labels if "SPI3" in l]
    cs = [l for l in labels if "CS" in l or "_SPI" in l]

    print("\n=== Problems ===")
    if typos:
        print(f"  Typo: {typos} -> rename to SPI1_MISO")
    if spi3:
        print(f"  SPI3 labels not in pinout IOC: {spi3}")
        print("    (PC10/11/12 conflict: PC12 is UART5_TX in IOC)")
    if cs:
        for l in cs:
            p = expected_port(l)
            print(f"  CS label {l} on {p}: not in pinout IOC (add GPIO_Output in Cube if used)")
    for row in wrong:
        print(f"  {row[0]} -> {row[1]}: {row[2]}")
    if unmapped:
        print(f"  Unmapped labels: {unmapped}")

    labeled_ports = {expected_port(l) for l in labels if expected_port(l)}
    print("\n=== IOC pins without function label on schematic ===")
    for p, s in sorted(ioc_pins.items()):
        if p in labeled_ports:
            continue
        if p in {"PB0", "PB1", "PB2", "PC5", "PE7", "PE8"}:
            note = "GPIO_Input — optional to use port name as label"
        elif p == "PA6":
            note = "ADC1_INP3 — consider ADC label"
        elif p == "PA10":
            note = "USB_OTG_FS_ID — consider USB_OTG_ID label"
        else:
            note = "missing label"
        print(f"  {p}: {s} ({note})")


if __name__ == "__main__":
    main()

"""
Example 3: Manual subsystem control

Demonstrates controlling weir, flow, and gauge individually — useful for
calibration, testing, or troubleshooting outside a full experiment run.
"""

from laguna import FlumeLab
from laguna.weir import SaflWeirController
from laguna.flow import SaflFlowController
from laguna.gauge import SaflWaterLevelSensor


def test_weir():
    print("=== Testing Weir ===")
    lab = FlumeLab()
    lab.add(SaflWeirController(lab.config.get("weir")))

    if not lab.weir.connect():
        print("Failed to connect to weir controller")
        return

    print("Homing weir...")
    lab.weir.home()

    print("Moving to 200 mm...")
    lab.weir.set_elevation(200.0)

    print(f"Weir status: {lab.weir.get_status()}")
    lab.weir.disconnect()
    print("Done!\n")


def test_flow():
    print("=== Testing Flow ===")
    lab = FlumeLab()
    lab.add(SaflFlowController(lab.config.get("flow")))

    if not lab.flow.connect():
        print("Failed to connect to flow controller")
        return

    lab.flow.qin = True
    lab.flow.start()
    lab.flow.set_flowrate(15.0)   # L/min

    print(f"Flow status: {lab.flow.get_status()}")

    lab.flow.stop()
    lab.flow.disconnect()
    print("Done!\n")


def test_gauge():
    print("=== Testing Gauge ===")
    lab = FlumeLab()
    lab.add(SaflWaterLevelSensor(lab.config.get("gauge")))

    if not lab.gauge.connect():
        print("Failed to connect to gauge sensor")
        return

    elevation = lab.gauge.read_mm()
    print(f"Water surface elevation: {elevation:.1f} mm")
    print(f"Gauge status: {lab.gauge.get_status()}")

    lab.gauge.disconnect()
    print("Done!\n")


def main():
    test_weir()
    test_flow()
    test_gauge()


if __name__ == "__main__":
    main()

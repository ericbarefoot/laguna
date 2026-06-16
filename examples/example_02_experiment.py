"""
Example 2: Simple experiment workflow

This example shows how to run a timed experiment with the weir and flow
subsystems connected and a schedule driving setpoints.
"""

from laguna import FlumeLab
from laguna.weir import SaflWeirController
from laguna.flow import SaflFlowController


def main():
    lab = FlumeLab(config_file="config/example_config.yaml")

    lab.add(SaflWeirController(lab.config.get("weir")))
    lab.add(SaflFlowController(lab.config.get("flow")))

    if not lab.connect_all():
        print("One or more subsystems failed to connect — aborting.")
        return

    with lab.experiment() as clock:
        print("Experiment started.")

        lab.weir.home()
        lab.weir.set_elevation(150.0)   # mm

        lab.flow.start()
        lab.flow.set_flowrate(20.0)     # L/min

        clock.wait_until(60.0)          # run for 60 s

        lab.flow.stop()

    lab.disconnect_all()
    print("Experiment complete.")


if __name__ == "__main__":
    main()

 # Workspace Instructions (serial_relay)

## Project Rule: Keep PCB Notes Current

In this project, whenever a new **hardware part** is added or an existing part’s wiring changes, you must update the **"PCB Capture Checklist (KiCad-Friendly)"** section in [execution_plan.md](execution_plan.md).

This includes, at minimum:
- Adding the part to the **Known Parts** list
- Updating **pin connections / net names** tables
- Updating any **power / level shifting** notes relevant to that part

If any part details are unknown (exact module pin order, I2C address, resistor values, etc.), record the assumption and add a TODO to confirm.

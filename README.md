Nut Milk Maker

Making plant milk at home usually means a blender, strainer bags and a pile of dishes. We wanted one machine that does it all: grind, heat, mix and filter in a single cycle. And when something breaks, you fix the part instead of throwing the whole thing away.

This is an open-source, repairable countertop nut milk maker that a local FabLab can build for under €200.

Status: concept and feasibility stage. We've reviewed the design, run the control logic in simulation and researched the parts, but we haven't built or tested a physical prototype yet.

What makes it different:

Three swappable modules: Motor, Blade and Jug. Change one without touching the rest.

Easy to repair: wear parts (seal, blade, coupling) swap in under 10 minutes, full teardown in 15, using no more than 3 standard tools.

FabLab friendly: printed, laser-cut and machined parts, plus off-the-shelf components.

Keeps working when a part fails: the firmware has a self-test (hold Start for 5 s) and workaround modes, for example timed heating if the temperature sensor dies. Nothing ever bypasses the lid interlock.

Simple controls: Start/Stop, Speed, Program, a Time knob and one LCD. One press loads a ready-made recipe.



	
Getting started

Read the report in /docs/Final.pdf first. Its BOM tables give the part names (MM01…, JM01…) used across this repo. Then you'll find CAD/STEP, STL, G-code, firmware, schematics and the BOM in their folders.



License

Released under the CERN Open Hardware Licence (CERN-OHL). Study it, change it, build it, share it.



Please be careful

This machine combines mains voltage, 80–100 °C liquid and a fast-spinning blade. It is a prototype for experienced makers, not a certified (CE) appliance, and it comes with no safety guarantee. Build and use it at your own risk, never run it unattended while testing, and never defeat the lid interlock.

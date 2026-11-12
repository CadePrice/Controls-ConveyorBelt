# Speed-controlled conveyor belt

Items drop onto a small conveyor driven by a 12 V gearmotor. The encoder on the
back of the motor counts how far the motor turns. The motor drives the roller
directly and the belt doesn't slip, so that tells the Arduino the belt speed.

## How to use it
1. In MATLAB, go to this folder (`ConveyorBelt_Simscape`).
2. Open a model and press **Run** (it runs 0 to 100 s, ode4, 0.01 s steps):
   - `conveyor_belt_openloop.slx`: no controller. The block under
     **Place Signal Here** sends a PWM duty (0–100 %) into the **Conveyor Belt**
     subsystem. Replace it with any signal you like. The Scope shows your command
     and the belt speed from the encoder. `out.speed_mms` (encoder belt speed)

The models read `CAD/conveyor_belt_params.m` every time they open or run, so
you never need to run it yourself.



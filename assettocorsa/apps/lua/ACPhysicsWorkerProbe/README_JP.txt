AC Physics Worker Probe v1.5.1
================================

This is the corrected final Physics Worker Probe.

Why v1.5.1 exists
-----------------
The original v1.5 completed its force-call sequence but recorded every
response sample as exactly zero. The vehicle itself still showed non-zero
motion, so the result was rejected as a measurement failure.

v1.5.1 corrects the measurement path by:

- waking the car before every positive or negative sign preparation;
- allowing a settling window after every wake;
- clearing pair samples before each new pair;
- judging polarity from positive-minus-negative separation rather than
  requiring the absolute vehicle velocity to cross zero;
- using a fresh shared-data namespace so v1.5 values cannot remain cached.

Test
----
Local-Z force magnitude: 80
Durations: 1, 2, 4 and 8 physics frames
Repeats: five positive/negative pairs per duration
Manual dt scaling: none

Installation
------------
Close Assetto Corsa and replace:

  assettocorsa/apps/lua/ACPhysicsWorkerProbe/

with the folder from this ZIP.

Conditions
----------
Extended Physics: ON
Vehicle: Kunos AE86
Track: Vallelunga
Mode: Offline Practice
Gear: Neutral
Speed: 0 km/h
Throttle/brake/handbrake: released

Procedure
---------
1. Start physics worker.
2. Confirm Alive=true, Physics allowed=true and Safety=READY.
3. ARM final duration test.
4. Within 10 seconds RUN armed final test.
5. Do not touch controls for roughly 40 seconds.
6. Wait for Phase=COMPLETE.
7. Export v1.5.1 report.
8. Stop physics worker.

Expected pass
-------------
DURATION LINEAR / DIRECT FORCE CONFIRMED
Calibrated: true

Safety
------
Maximum magnitude is raw 80.
Maximum continuous application is eight physics frames.
No wheel torque, RPM, controls, tyres or mass are changed.

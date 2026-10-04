# Nezbot Calibrate

A MakeCode extension that measures a Nezha robot's wheels and track, for
robots driven by the
[DiffDrive](https://github.com/League-Robotics/pxt-nezha-diffdrive)
extension.

It needs a micro:bit **V2**, a Nezha brick with two motors and a PlanetX
Trackbit line sensor.

## Add it to a project

In MakeCode, open **Extensions** and paste:

> https://github.com/League-Robotics/pxt-nezbot-calibrate

## Use it

```typescript
calibrate.registerPrograms()
```

That one call brings the robot up and adds four programs. Press **A** to
step through them, **B** to run the one shown, and any button to stop a
running program. Each program is also a run command of the same name.

| Program | Picture | What it does |
| --- | --- | --- |
| `circle` | circle | Drives a 30 cm radius circle, to check a calibration |
| `square` | square | Drives a 50 cm square, to check a calibration |
| `calwheels` | out and back | Drives between two lines a measured distance apart and works out the wheel size |
| `calturn` | spin | Spins on the iron cross and works out the track width and slip |

`calwheels` takes the distance between the lines in cm and, if you know
it, the wheel diameter in mm: `RUN calwheels 90.5 0`. `calturn` takes
the number of edges to time: `RUN calturn 10`.

A calibration that succeeds is used straight away and kept in the
robot's flash memory, so it is still there after a reset or after a
different program is flashed. `diffDrive.setupRobot()` loads it at
start-up.

## Read the calibration back

```typescript
calibrate.dump()
```

writes the calibration the robot is driving on to the serial port as one
JSON line:

```
{"ev":"cal.values","name":"tovez","wheel":0.7842,"diameter":89.86,"tw":11.14,"slip":0.998,"scale_l":1,"scale_r":1,"port_l":2,"port_r":1}
```

| Field | Meaning |
| --- | --- |
| `wheel` | wheel travel per shaft degree, mm |
| `diameter` | wheel diameter, mm |
| `tw` | track width, cm |
| `slip` | rotational slip |
| `scale_l`, `scale_r` | wheel multipliers |
| `port_l`, `port_r` | brick port of each motor |

```typescript
calibrate.dumpCode()
```

writes the same values as code to paste into a program:

```typescript
// calibration for tovez
diffDrive.setMotorPorts(MotorPort.M2, MotorPort.M1)
diffDrive.setWheelMultiplier(MotorSide.Left, 1)
diffDrive.setWheelMultiplier(MotorSide.Right, 1)
diffDrive.setWheelCalibration(0.7842)
diffDrive.setTrackWidth(11.14)
diffDrive.setConfigValue(ConfigField.RotationalSlip, 0.998)
```

Both are also run commands, `RUN _caldump` and `RUN _calcode`.

## License

MIT

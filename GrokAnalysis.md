Our FRC robot swerve drives work correctly when we drive with no load. Once we start to drive fast and hard one of the swerve steer motor is not aligned. Attached is out code.

Your `SwerveSubsystem` has this:

```java
m_swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
```

YAGSL initializes the SparkMax relative encoder from the CANcoder at startup, then runs closed-loop on the relative encoder. With auto-sync disabled, any mechanical slip, backlash, or magnet movement under high torque causes the relative encoder to drift from the true wheel angle. Hard driving creates exactly those forces (drive motors trying to twist the modules).

### Quick software fixes to try first

1. **Enable auto-synchronize** (most common fix for this symptom):
   ```java
   m_swerveDrive.setModuleEncoderAutoSynchronize(true, 2.0);  // deadband in degrees
   ```
   This re-syncs relative → absolute whenever the module is nearly stationary.

2. **Reduce the angle motor ramp rate** in `physicalproperties.json` so the steer can correct faster under load:
   ```json
   "rampRate": {
     "drive": 0.25,
     "angle": 0.05   // or 0
   }
   ```

3. **Slightly raise angle P** (yours is 0.009). Many teams run ~0.01–0.015 on SparkMax relative feedback. Increase carefully and watch for oscillation.

### Critical mechanical checks (especially on the failing module)

- **CANcoder magnet** — Is it secured with Loctite 609 or epoxy? Unsecured magnets are the #1 cause of “works on the bench / fails when you drive hard.”
- Gear mesh / backlash in the azimuth stage.
- Pinion set screw + Loctite on the SparkMax shaft.
- Rigid mounting of the CANcoder itself.

### Diagnostic (do this)

In AdvantageScope or Elastic, log for the bad module:
- SparkMax relative position (converted to degrees)
- CANcoder absolute position
- Desired module angle

Drive hard and watch whether the relative and absolute diverge.  
- **They diverge** → mechanical slip or magnet movement.  
- **They stay together but the wheel is still wrong** → torque/PID/current limit issue.

### Other notes from your configs

- Angle current limit of 40 A is fine for a NEO; a bit high for a NEO 550. Confirm which motor you’re using.
- Angle gear ratio of 18.75 is non-standard (MK4 = 12.8, MK4i ≈ 21.43). Double-check it’s correct for your modules—wrong ratio produces progressive error that looks like misalignment after lots of rotation.
- All four modules have `inverted.angle: true` and unique absolute offsets, which is normal.

Try enabling auto-sync first and securing the magnet on the failing module. Those two changes fix this exact symptom for a large number of teams. If you can tell me which module is failing and what physical modules you’re running (MK4i, MaxSwerve, etc.), I can look more closely at the gear ratio and offsets.
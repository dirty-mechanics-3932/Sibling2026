package frc.robot.subsystems.intake;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTilt extends SubsystemBase {

    public final TalonFX tiltMotor; // gear ratio = 52.5
    private final TalonFXConfiguration config;
    private final MotionMagicVoltage motionMagicVoltage;
    // private final TorqueCurrentFOC homingCurrent;
    @Logged(name = "IntakeTilt Limit Switch")
    private final DigitalInput limitSwitch = new DigitalInput(9);
    // Range of motion, in degrees
    @Logged(name = "IntakeTilt Min Angle")
    public static final double MIN_ANGLE = 0.0;
    @Logged(name = "IntakeTilt Max Angle")
    public static final double MAX_ANGLE = 135.0;
    @Logged(name = "IntakeTilt Extend Angle")
    private static final double EXTEND_ANGLE = 120.0;
    @Logged(name = "IntakeTilt Gear Ratio")
    private static final double GEAR_RATIO = 52.5;
    @Logged(name = "IntakeTilt Tolerance")
    private double toleranceDeg = 3.0;

    // Motion Magic tuning (in motor rotations/sec, rotations/sec^2,
    // rotations/sec^3)
    private double cruiseVelocityRps = 100.0;
    private double accelerationRpsPerSec = 200.0;
    private double jerkRpsPerSec2 = 0.0; //800.0; // 0 disables jerk limiting (trapezoidal only)
    // private double bounceIntakeAngle = 80.0; // make method for this later

    // Current limiting
    private double statorCurrentLimitAmps = 40.0; // hard safety limit for all motion
    private double supplyCurrentLimitAmps = 30.0; // limits draw from the battery/PDH
    // private double homingCurrentAmps = 8.0; // 8.0; // gentle, fixed current used
    // while homing

    double motorRotations;
    @Logged(name = "Last Pose Deg")
    private double lastPositionDeg = 0.0;
    private boolean homing = false, homed = false;
    @Logged
    private int zeroEncoderCount = 0;

    public IntakeTilt() {
        zeroEncoderCount = 0;
        tiltMotor = new TalonFX(21); // Remember to set the motor IDs
        config = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0);
        // homingCurrent = new TorqueCurrentFOC(0);

        config.Slot0.kP = 4.5;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;

        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityRps;
        config.MotionMagic.MotionMagicAcceleration = accelerationRpsPerSec;
        config.MotionMagic.MotionMagicJerk = jerkRpsPerSec2;

        config.CurrentLimits.StatorCurrentLimit = statorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        tiltMotor.getConfigurator().apply(config);
        if (getLimitSwitch()) {
            endHoming();
        } else {
            homeInPlace();
        }
    }

    // Returns true if the limit switch is pressed
    // limitSwitch.get() returns false when pressed, so we invert it
    @Logged(name = "IntakeTilt Limit Switch Triggered")
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void zeroEncoder() {
        tiltMotor.setPosition(0);
        logf("Zeroed Intake Encoder");
    }

    public Command setIntakeTiltSetpointDeg(double value) {
        return Commands.runOnce(() -> moveIntakeDeg(value));
    }

    public Command extendIntake() {
        return setIntakeTiltSetpointDeg(EXTEND_ANGLE);
    }

    public Command moveIntakeTiltDeltaDegCmd(double value) {
        return Commands.runOnce(() -> moveIntakeDeg(lastPositionDeg + value));
    }

    public void moveIntakeTiltDeltaDeg(double value) {
        moveIntakeDeg(lastPositionDeg + value);
    }

    public void extend() {
        moveIntakeDeg(EXTEND_ANGLE);
        tiltMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void moveIntakeDeg(double degrees) {
        if (!homed) {
            logf("********  Tried to move intake it was not homed");
            return;
        }
        degrees = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
        lastPositionDeg = degrees;
        double rots = degrees * GEAR_RATIO / 360.0;
        tiltMotor.setControl(motionMagicVoltage.withPosition(rots).withEnableFOC(true));
    }

    @Logged(name = "IntakeTilt getPositionDeg")
    public double getPositionDeg() {
        return tiltMotor.getPosition().getValueAsDouble() * 360 / GEAR_RATIO;
    }

    public void homeIntake() {
        if (!homing && !getLimitSwitch()) {
            tiltMotor.set(-0.15);
            tiltMotor.setNeutralMode(NeutralModeValue.Brake);
            homing = true;
            homed = false;
            logf("Starting Intake Homing");
        } else {
            endHoming();
        }
    }

    public void homeInPlace() {
        logf("**** Home in place");
        homed = true;
        homing = false;
        lastPositionDeg = EXTEND_ANGLE;
        tiltMotor.setPosition(EXTEND_ANGLE * GEAR_RATIO / 360.0);
        tiltMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void endHoming() {
        // limitSwitch get return false when at limit
        tiltMotor.set(0);
        tiltMotor.setNeutralMode(NeutralModeValue.Brake);
        lastPositionDeg = 0;
        homing = false;
        homed = true;
        zeroEncoderCount = 8; // wait a few cycles before zeroing the encoder
        logf("Ending Intake Homing pos%.2f", getPositionDeg());
    }

    public boolean isAtTarget() {
        return Math.abs(lastPositionDeg - getPositionDeg()) < toleranceDeg;
    }

    @Override
    public void periodic() {
        if (zeroEncoderCount > 0) {
            zeroEncoderCount--;
            if (zeroEncoderCount == 0) {
                zeroEncoder();
            }
        }
        // if (Robot.count % 20 == 5) {
        //     SmartDashboard.putNumber("IntakeTiltPos", getPositionDeg());
        //     SmartDashboard.putNumber("IntakeTiltDeg", lastPositionDeg);
        //     SmartDashboard.putBoolean("IntakeLimit", !limitSwitch.get());
        //     SmartDashboard.putBoolean("IntakeHomed", homed);
        // }
        // if (Robot.count % 100 == 0) {
            // logf("Intake pos:%.2f target:%.2f limit:%b atSet:%b homed:%b current:%.2f",
            // getPositionMotorDeg(),
            // lastPositionDeg,
            // getLimitSwitch(), isAtTarget(), homed,
            // tiltMotor.getSupplyCurrent().getValueAsDouble());
        // }
        if (homing && getLimitSwitch()) {
            endHoming();
        }
    }
}

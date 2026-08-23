package frc.robot.subsystems.intake;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeTilt extends SubsystemBase {

    public final TalonFX tiltMotor; // gear ratio = 52.5
    private final TalonFXConfiguration config;
    private final MotionMagicVoltage motionMagicVoltage;
    private final TorqueCurrentFOC homingCurrent;
    private final DigitalInput limitSwitch = new DigitalInput(9);
    // Range of motion, in degrees
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 150.0;
    private double gearRatio = 52.5;
    private double toleranceDeg = 3.0;

    // Motion Magic tuning (in motor rotations/sec, rotations/sec^2,
    // rotations/sec^3)
    private double cruiseVelocityRps = 20.0;
    private double accelerationRpsPerSec = 40.0;
    private double jerkRpsPerSec2 = 400.0; // 0 disables jerk limiting (trapezoidal only)
    private double bounceIntakeAngle = 80.0; // make method for this later

    // Current limiting
    private double statorCurrentLimitAmps = 40.0; // hard safety limit for all motion
    private double supplyCurrentLimitAmps = 30.0; // limits draw from the battery/PDH
    private double homingCurrentAmps = 8.0; // 8.0; // gentle, fixed current used while homing

    double motorRotations;
    private double lastPositionDeg;
    private boolean homing = false, homed = false;
    private int zeroEncoderCount = 0;

    public IntakeTilt() {
        tiltMotor = new TalonFX(21); // Remember to set the motor IDs
        config = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0);
        homingCurrent = new TorqueCurrentFOC(0);

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
        }
    }

    // Returns true if the limit switch is pressed
    // limitSwitch.get() returns false when pressed, so we invert it
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public void zeroEncoder() {
        tiltMotor.setPosition(0);
    }

    public Command setIntakeTiltSetpointDeg(double value) {
        return Commands.runOnce(() -> moveIntakeDeg(value));
    }

    public Command extendIntake() {
        return setIntakeTiltSetpointDeg(118);
    }

    public Command moveIntakeTiltDeltaDeg(double value) {
        return Commands.runOnce(() -> moveIntakeDeg(lastPositionDeg + value));
    }

    private void moveIntakeDeg(double degrees) {
        if (!homed) {
            logf("********  Tried to move intake it was not homed");
            return;
        }
        degrees = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
        lastPositionDeg = degrees;
        double rots = degrees * gearRatio / 360.0;
        tiltMotor.setControl(motionMagicVoltage.withPosition(rots).withEnableFOC(true));
    }

    public double getPositionMotorDeg() {
        return tiltMotor.getPosition().getValueAsDouble() * 360 / gearRatio;
    }

    public void homeIntake() {
        if (!homing && !getLimitSwitch()) {
            tiltMotor.set(-0.15);
            homing = true;
            homed = false;
            logf("Starting Intake Homing");
        } else {
            endHoming();
        }
    }

    public void endHoming() {
        // limitSwitch get return false when at limit
        tiltMotor.set(0);
        lastPositionDeg = 0;
        homing = false;
        homed = true;
        zeroEncoderCount = 8; // wait a few cycles before zeroing the encoder
        logf("Ending Intake Homing pos%.2f", getPositionMotorDeg());
    }

    public boolean isAtTarget() {
        return Math.abs(lastPositionDeg - getPositionMotorDeg()) < toleranceDeg;
    }

    // public Command extendIntake(double degrees) {
    // lastPosition = degrees;
    // motorRotations = gearRatio * (degrees/360);
    // return Commands.runOnce(()->setIntakeTiltSetpoint(motorRotations));
    // }

    // public Command bounceIntake(double degrees) {
    // lastPosition = degrees;
    // motorRotations = gearRatio * (degrees/360);
    // return Commands.runOnce(()->setIntakeTiltSetpoint(motorRotations));
    // }

    @Override
    public void periodic() {
        if (zeroEncoderCount > 0) {
            zeroEncoderCount--;
            if (zeroEncoderCount == 0) {
                zeroEncoder();
                logf("Zeroed Intake Encoder");
            }
        }
        if (Robot.count % 20 == 5) {
            SmartDashboard.putNumber("IntakeTiltPos", getPositionMotorDeg());
            SmartDashboard.putNumber("IntakeTiltDeg", lastPositionDeg);
            SmartDashboard.putBoolean("IntakeLimit", !limitSwitch.get());
        }
        if (Robot.count % 100 == 0) {
            // logf("Intake pos:%.2f  target:%.2f limit:%b atSet:%b homed:%b current:%.2f", getPositionMotorDeg(),
            //         lastPositionDeg,
            //         getLimitSwitch(), isAtTarget(), homed, tiltMotor.getSupplyCurrent().getValueAsDouble());
        }
        if (homing && getLimitSwitch()) {
            endHoming();
        }
    }
}

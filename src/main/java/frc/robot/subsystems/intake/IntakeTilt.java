package frc.robot.subsystems.intake;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeTilt extends SubsystemBase {

    public final TalonFX m_intakeTiltMotor; // gear ratio = 52.5
    private final TalonFXConfiguration config;
    private final PositionVoltage positionVoltage;
    private final DigitalInput limitSwitch = new DigitalInput(9);
    // Range of motion, in degrees
    public static final double MIN_ANGLE = -50.0;
    public static final double MAX_ANGLE = 0.0;
    private double gearRatio = 52.5;
    private double toleranceDeg = 3.0;
    double motorRotations;
    private double lastPositionDeg;
    private boolean homing = false, homed = false;

    public IntakeTilt() {
        m_intakeTiltMotor = new TalonFX(21); // Remember to set the motor IDs
        config = new TalonFXConfiguration();
        positionVoltage = new PositionVoltage(0);
        config.Slot0.kP = 4.5;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
        m_intakeTiltMotor.getConfigurator().apply(config);
    }

    public void zeroEncoder() {
        m_intakeTiltMotor.setPosition(0);
    }

    public Command setIntakeTiltSetpointDeg(double value) {
        return Commands.runOnce(() -> moveIntakeDeg(value));
    }

    public void moveIntakeDeg(double degrees) {
        if(homing || !homed) {
            return;
        }
        degrees = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
        lastPositionDeg = degrees;
        double rots = degrees * gearRatio / 360.0;
        m_intakeTiltMotor.setControl(positionVoltage.withPosition(rots).withEnableFOC(true));
    }

    public double getPositionMotorDeg() {
        return m_intakeTiltMotor.getPosition().getValueAsDouble() * 360 / gearRatio;
    }

    public void homeIntake() {
        if (!homing && limitSwitch.get()) {
            m_intakeTiltMotor.set(-.1);
            homing = true;
            homed = false;
        } else {
            endHoming();
        }
    }

    public void endHoming() {
        // limitSwitch get return false when at limit
        m_intakeTiltMotor.set(0);
        logf("Intake Limit Switch Hit");
        zeroEncoder();
        lastPositionDeg = 0;
        homing = false;
        homed = true;
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
        if (Robot.count % 20 == 5) {
            SmartDashboard.putNumber("IntakeTiltPos", getPositionMotorDeg());
            SmartDashboard.putNumber("IntakeTiltDeg", lastPositionDeg);
            SmartDashboard.putBoolean("IntakeLimit", !limitSwitch.get());
            SmartDashboard.putBoolean("IntakeAtSetpoint", isAtTarget());
        }
        if (Robot.count % 100 == 0) {
            logf("Intake pos:%.2f  target:%.2f limit:%b atSet:%b homed:%b", getPositionMotorDeg(), lastPositionDeg,
                    !limitSwitch.get(), isAtTarget(), homed);
        }
        if (homing && !limitSwitch.get()) {
            endHoming();
        }
    }
}

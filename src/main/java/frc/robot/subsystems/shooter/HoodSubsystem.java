package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class HoodSubsystem extends SubsystemBase {
    public final TalonFX m_hoodMotor;
    private final CANcoder absoluteEncoder;
    private final TalonFXConfiguration config;
    private final MotionMagicVoltage motionMagicVoltage;
    private final CANcoderConfiguration configEncoder;
    private double targetPositionRot;
    private double toleranceDeg = 2; // degrees

    // Range of motion, in degrees
    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(50.0);

    public static final Angle MIN_ROTATION = MIN_ANGLE;
    public static final Angle MAX_ROTATION = MAX_ANGLE.in(Rotations);

    // Motor rotations per 1 CANcoder rotation (CANcoder is mounted on the hood
    // output shaft)
    double rotorToSensorRatio = 4.5454;

    public HoodSubsystem() {
        m_hoodMotor = new TalonFX(50); // Remember to set the motor IDs
        absoluteEncoder = new CANcoder(51);
        config = new TalonFXConfiguration();
        configEncoder = new CANcoderConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0);

        configEncoder.MagnetSensor.MagnetOffset = 0.563476;

        config.Slot0.kP = 20.0;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.Feedback.RotorToSensorRatio = rotorToSensorRatio; // motor -> CANcoder reduction
        config.Feedback.SensorToMechanismRatio = 1; // CANcoder is already on the mechanism

        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // Motion Magic profile constraints — tune these for your mechanism.
        // Units are rotations/sec, rotations/sec^2, rotations/sec^3 (mechanism
        // rotations, since
        // SensorToMechanismRatio = 1 here, i.e. CANcoder rotations).
        config.MotionMagic.MotionMagicCruiseVelocity = 15.0; // max velocity RPS
        config.MotionMagic.MotionMagicAcceleration = 30.0; // time to reach cruise velocity rps^2
        config.MotionMagic.MotionMagicJerk = 40.0; // time to reach max accel (0 = disabled)

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ROTATION.in(Degrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ROTATION.in(Degrees);

        m_hoodMotor.getConfigurator().apply(config);
        absoluteEncoder.getConfigurator().apply(configEncoder);
    }

    public double getHoodPositionInDeg() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public Command setHoodPosition(Angle valueDeg) {
        return Commands
                .run(() -> setPositionWithEncoder(Degrees.of(MathUtil.clamp(valueDeg.in(Degrees), MIN_ROTATION.in(Degrees), MAX_ROTATION.in(Degrees)))), this)
                .until(() -> atPosition(Degrees.of(MathUtil.clamp(valueDeg.in(Degrees), MIN_ROTATION.in(Degrees), MAX_ROTATION.in(Degrees)))));
    }

    public void setPositionWithEncoder(Angle valueRot) {
        targetPositionRot = MathUtil.clamp(valueRot.in(Rotations), MIN_ROTATION.in(Degrees), MAX_ROTATION.in(Degrees));
        m_hoodMotor.setControl(motionMagicVoltage.withPosition(targetPositionRot));
    }

    public Command stop() {
        return Commands.runOnce(() -> m_hoodMotor.stopMotor());
    }

    public boolean atPosition(Angle targetRot) {
        double targetDeg = targetRot.in(Degrees);
        SmartDashboard.putNumber("Hoodpos - target", Math.abs(getHoodPositionInDeg() - targetDeg));
        return Math.abs(getHoodPositionInDeg() - targetDeg) <= toleranceDeg;
    }

    @Override
    public void periodic() {
        if (Robot.count % 20 == 6) {
            SmartDashboard.putNumber("Hood Position", getHoodPositionInDeg());
            SmartDashboard.putNumber("Hood Target", targetPositionRot * 360);
            SmartDashboard.putNumber("Tolerance", toleranceDeg);
            SmartDashboard.putBoolean("Hood At Position", atPosition(Rotations.of(targetPositionRot)));
        }
        if (Robot.count % 1000 == 0) {
            logf("Hood Position: %.2f, Target: %.2f, At Position: %b", getHoodPositionInDeg(), targetPositionRot * 360,
                    atPosition(Rotations.of(targetPositionRot)));
        }
    }
}
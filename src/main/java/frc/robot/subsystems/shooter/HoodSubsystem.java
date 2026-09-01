package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
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
    @Logged
    private Angle targetPositionRot = Rotations.of(0.0);
    @Logged
    private Angle targetPositionDeg = Degrees.of(0.0);
    @Logged
    private Angle toleranceDeg = Degrees.of(0.5); // degrees
    @Logged(name = "Hood atTarget")
    private boolean atTarget = false;

    // Range of motion, in degrees
    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(50.0);

    // Motor rotations per 1 CANcoder rotation (CANcoder is mounted on the hood output shaft)
    double rotorToSensorRatio = 4.5454;

    public HoodSubsystem() {
        m_hoodMotor = new TalonFX(50);
        absoluteEncoder = new CANcoder(51);
        config = new TalonFXConfiguration();
        configEncoder = new CANcoderConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0);

        configEncoder.MagnetSensor.MagnetOffset = 0.563476;

        config.Slot0.kP = 75.0;
        config.Slot0.kI = 0.0009;
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
        config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // max velocity RPS
        config.MotionMagic.MotionMagicAcceleration = 200.0; // time to reach cruise velocity rps^2
        config.MotionMagic.MotionMagicJerk = 0.0; // time to reach max accel (0 = disabled)

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotation);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotation);

        m_hoodMotor.getConfigurator().apply(config);
        absoluteEncoder.getConfigurator().apply(configEncoder);
    }

    @Logged(name = "Hood Pos Deg")
    public Angle getHoodPositionInDeg() {
        return Degrees.of(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Command setHoodPosition(Angle valueDeg) {
        return Commands
                .run(() -> setPositionWithEncoder(valueDeg), this)
                .until(() -> hoodAtPosition(valueDeg));
    }

    public Angle setPositionWithEncoder(Angle valueDeg) {
        //SmartDashboard.putNumber("Hood is trying to go here", valueRot*360); 
        targetPositionDeg = Degrees.of(MathUtil.clamp(valueDeg.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));
        targetPositionRot = Rotation.of(targetPositionDeg.in(Rotation));
        m_hoodMotor.setControl(motionMagicVoltage.withPosition(targetPositionRot));
        return targetPositionRot;
    }

    public Command stop() {
        return Commands.runOnce(() -> m_hoodMotor.stopMotor());
    }

    public boolean hoodAtPosition(Angle targetDeg) {
        //SmartDashboard.putNumber("Hoodpos  Error", Math.abs(getHoodPositionInDeg() - targetDeg));
        atTarget = getHoodPositionInDeg().isNear(targetDeg, toleranceDeg);
        return getHoodPositionInDeg().isNear(targetDeg, toleranceDeg);
    }

    public boolean hoodAtTarget() {
        return hoodAtPosition(targetPositionRot);
    }

    @Override
    public void periodic() {
        if (Robot.count % 20 == 6) {
            //SmartDashboard.putNumber("Hood Position", round2(getHoodPositionInDeg()));
            //SmartDashboard.putNumber("Hood Target H", round2(targetPositionRot * 360));
            //SmartDashboard.putNumber("Tolerance", toleranceDeg);
            //SmartDashboard.putBoolean("Hood At Position", hoodAtPosition(targetPositionRot));
        }
        // if (Robot.count % 1000 == 0) {
        //     logf("Hood Position: %.2f, Target: %.2f, At Position: %b", getHoodPositionInDeg(), targetPositionRot * 360,
        //             hoodAtPosition(targetPositionRot));
        // }
    }
}
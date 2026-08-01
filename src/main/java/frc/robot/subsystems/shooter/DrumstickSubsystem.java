package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrumstickSubsystem extends SubsystemBase {

    public final TalonFX m_velocityLeader;
    public final TalonFX m_velocityFollower1;
    public final TalonFX m_velocityFollower2;
    
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;

    public DrumstickSubsystem() {
        // Remember to set the motor IDs
        m_velocityLeader = new TalonFX(31); 
        m_velocityFollower1 = new TalonFX(32);
        m_velocityFollower2 = new TalonFX(33);

        configs = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;

        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        m_velocityLeader.getConfigurator().apply(configs);
        m_velocityFollower1.getConfigurator().apply(configs);
        m_velocityFollower2.getConfigurator().apply(configs);

        m_velocityFollower1.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        m_velocityFollower2.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setShooterSetpoint(double value) {
        m_velocityLeader.setControl(velocityRequest.withVelocity(value).withEnableFOC(true));
    }

    public double getVelocity() {
        return m_velocityLeader.getVelocity().getValueAsDouble();
    }

    public void stopShooter() {
        m_velocityLeader.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drumstick Velocity", getVelocity());
    }
}

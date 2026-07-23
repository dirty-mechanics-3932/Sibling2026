package frc.robot.subsystems.catchup;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatchupSubsystem extends SubsystemBase {

    public final TalonFX m_velocityLeader;
    public final TalonFX m_velocityFollower1;
    public final TalonFX m_velocityFollower2;
    
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;

    public CatchupSubsystem() {
        // Remember to set the motor IDs
        m_velocityLeader = new TalonFX(1); 
        m_velocityFollower1 = new TalonFX(2);
        m_velocityFollower2 = new TalonFX(3);

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

        m_velocityFollower1.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        m_velocityFollower2.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setVelocitySetpoint(double value) {
        m_velocityLeader.setControl(velocityRequest.withVelocity(value).withEnableFOC(true));
    }

    public double getVelocityMotor() {
        return m_velocityLeader.getVelocity().getValueAsDouble();
    }

    public void stop() {
        m_velocityLeader.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Catchup Velocity", getVelocityMotor());
    }
}

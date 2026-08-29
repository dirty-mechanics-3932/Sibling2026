package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatchupSubsystem extends SubsystemBase {

    public final TalonFX m_velocityLeader;
    public final TalonFX m_velocityFollower;
    
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;


    public CatchupSubsystem() {
        m_velocityLeader = new TalonFX(41); 
        m_velocityFollower = new TalonFX(40);

        configs = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;

        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        m_velocityLeader.getConfigurator().apply(configs);
        m_velocityFollower.getConfigurator().apply(configs);

        m_velocityFollower.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public Command setCatchupSetpointCmd(double value) {
        return Commands.runOnce(()->m_velocityLeader.setControl(velocityRequest.withVelocity(value/60).withEnableFOC(true)));
    }

    public void setCatchupSetpoint(double value){
        m_velocityLeader.setControl(velocityRequest.withVelocity(value/60).withEnableFOC(true)); 
    }


    public double getVelocity() {
        return m_velocityLeader.getVelocity().getValueAsDouble() * 60;
    }

    public boolean atSpeed(double target) {
        double tolerance = 1.0;
        return Math.abs(getVelocity() - target) < tolerance;
    }

    public Command stopCatchup() {
        return Commands.runOnce(()->m_velocityLeader.stopMotor());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Catchup Velocity", getVelocity());
    }
}

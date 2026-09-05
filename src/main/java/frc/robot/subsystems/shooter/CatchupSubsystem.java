package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatchupSubsystem extends SubsystemBase {

    public final TalonFX m_velocityLeader;
    public final TalonFX m_velocityFollower;
    
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;

    private final AngularVelocity tolerance = RPM.of(100.0);
    @Logged(name = "Catchup atSpeed")
    public boolean atSpeed = false;


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

    public Command setCatchupSetpointRPMCmd(AngularVelocity valueRPM) {
        return Commands.runOnce(()->m_velocityLeader.setControl(velocityRequest.withVelocity(valueRPM.in(RotationsPerSecond)).withEnableFOC(true)));
    }

    public AngularVelocity setCatchupSetpoint(AngularVelocity valueRPM){
        m_velocityLeader.setControl(velocityRequest.withVelocity(valueRPM.in(RotationsPerSecond)).withEnableFOC(true)); 
        return valueRPM;
    }

    @Logged(name = "Catchup gteVelocityRPS")
    public AngularVelocity getVelocityRPS() {
        return RotationsPerSecond.of(m_velocityLeader.getVelocity().getValueAsDouble());
    }

   // @Logged(name = "Catchup atSpeed") Cant log functions with arguments
    public boolean atSpeed(AngularVelocity targetRPS) {
        atSpeed = getVelocityRPS().isNear(targetRPS, tolerance);
        return getVelocityRPS().isNear(targetRPS, tolerance);
    }

    public Command stopCatchup() {
        return Commands.runOnce(()->m_velocityLeader.stopMotor());
    }

    public void stopMotor() {
        m_velocityLeader.stopMotor();
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Catchup Velocity", getVelocity());
    }
}

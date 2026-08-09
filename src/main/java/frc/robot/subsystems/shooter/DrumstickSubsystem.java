package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DrumstickSubsystem extends SubsystemBase {

    private final TalonFX m_velocityLeader = new TalonFX(31);
    private final TalonFX m_velocityFollower1;
    private final TalonFX m_velocityFollower2;
    
    private final VoltageOut m_voltReq = new VoltageOut(0.0); 
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;
    private double setpoint; 

    public DrumstickSubsystem() {
        // Remember to set the motor IDs
      //  m_velocityLeader = new TalonFX(31); 
        m_velocityFollower1 = new TalonFX(32);
        m_velocityFollower2 = new TalonFX(33);

        configs = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        configs.Slot0.kP = 0.0001;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.03;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;

        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        m_velocityLeader.getConfigurator().apply(configs);
        m_velocityFollower1.getConfigurator().apply(configs);
        m_velocityFollower2.getConfigurator().apply(configs);

        m_velocityFollower1.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        m_velocityFollower2.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public Command setShooterSetpoint(double value) {
        setpoint = value/60; 
        return Commands.runOnce(()->m_velocityLeader.setControl(velocityRequest.withVelocity(setpoint).withEnableFOC(true)));
    }

    public double getVelocity() {
        return m_velocityLeader.getVelocity().getValueAsDouble() * 60;
    }

    public boolean atSpeed(double target) {
        double tolerance = 1.0;
        return Math.abs(getVelocity() - target) < tolerance;
    }

    public Command stopShooter() {
        return Commands.runOnce(()->m_velocityLeader.stopMotor());
    }

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
             (state) -> SignalLogger.writeString("state", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (volts) -> m_velocityLeader.setControl(m_voltReq.withOutput(volts.in(Volts))),
            null,
            this
        )
    );

public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return sysIdRoutine.dynamic(direction);
}
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drumstick Velocity", getVelocity());
    }
}

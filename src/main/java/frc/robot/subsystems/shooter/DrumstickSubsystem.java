package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrumstickSubsystem extends SubsystemBase {

    private final TalonFX m_velocityLeader;
    private final TalonFX m_velocityFollower1;
    private final TalonFX m_velocityFollower2;
    
   private final VoltageOut m_voltReq = new VoltageOut(0.0); 
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;
    private double tolerance = 500;
     private AngularVelocity targetRPM = RPM.of(0);

    public DrumstickSubsystem() {
        // Remember to set the motor IDs
        m_velocityLeader = new TalonFX(31); 
        m_velocityFollower1 = new TalonFX(32);
        m_velocityFollower2 = new TalonFX(33);

        configs = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        configs.Slot0.kP = 0.0001;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.0; // Was 0.03
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        m_velocityLeader.getConfigurator().apply(configs);
        m_velocityFollower1.getConfigurator().apply(configs);
        m_velocityFollower2.getConfigurator().apply(configs);

        m_velocityFollower1.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        m_velocityFollower2.setControl(new Follower(m_velocityLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // public void setShooterSetpoint(double value) {
    //    m_velocityLeader.setControl(velocityRequest.withVelocity(value/60).withEnableFOC(true));
    //     SmartDashboard.putBoolean("Drumstick At Speed", atSpeed(value/60));
    // }

    public void setVelocityRPM(AngularVelocity rpm) {
        m_velocityLeader.setControl(velocityRequest.withVelocity(rpm).withEnableFOC(true));
        targetRPM = rpm;
    }

    // This command will run the shooter until it reaches the target speed, then return.
    public Command runToSpeed(AngularVelocity rpm) {
        return Commands.run(() -> setVelocityRPM(rpm), this).until(() -> atSpeed(rpm));
    }


    // public Command shootCommand(double value){
    //     return runEnd(()->setShooterSetpoint(value), ()->setShooterSetpoint(value)).until(()->atSpeed(value));
    // }

    public double getVelocityRPM() {
        return m_velocityLeader.getVelocity().getValueAsDouble() * 60;
    }

    public boolean atSpeed(AngularVelocity target) {
        return Math.abs(getVelocityRPM() - target.in(RPM)) <= tolerance;
    }

    // Overload so callers can pass the AngularVelocity-typed targetRPM directly.


    public Command stopShooter() {
        return Commands.runOnce(()->m_velocityLeader.stopMotor());
    }

//     private SysIdRoutine sysIdRoutine = new SysIdRoutine(
//         new SysIdRoutine.Config(
//             null,
//             Volts.of(4),
//             null,
//              (state) -> SignalLogger.writeString("state", state.toString())
//         ), 
//         new SysIdRoutine.Mechanism(
//             (volts) -> m_velocityLeader.setControl(m_voltReq.withOutput(volts.in(Volts))),
//             null,
//             this
//         )
//     );

// public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//    return sysIdRoutine.quasistatic(direction);
// }

// public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//    return sysIdRoutine.dynamic(direction);
// }
    @Override
    public void periodic() {
        if(Robot.count % 10 == 0) {
            SmartDashboard.putNumber("Drum Velocity", getVelocityRPM());
            SmartDashboard.putBoolean("Drum At Speed", atSpeed(targetRPM));
        }
        if (Robot.count % 100 == 0 && Math.abs(getVelocityRPM()) != 0) {
            logf("Drum Velocity:%.2f, At Speed:%b Target:%.2f",
                    getVelocityRPM(), atSpeed(targetRPM), targetRPM.in(RPM));
        }
        
    }
}
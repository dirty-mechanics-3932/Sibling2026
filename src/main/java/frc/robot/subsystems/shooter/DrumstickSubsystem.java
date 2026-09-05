package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrumstickSubsystem extends SubsystemBase {

    private final TalonFX m_velocityLeader;
    private final TalonFX m_velocityFollower1;
    private final TalonFX m_velocityFollower2;
    @SuppressWarnings("unused")
    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;
    @Logged(name = "Drumstick RPM Tolerance")
    private AngularVelocity tolerance = RPM.of(500);
    @Logged(name = "Drumstick targetRPM")
    private AngularVelocity targetRPM = RPM.of(0);
    @Logged(name = "Drumstick atTarget")
    private boolean atTarget = false;

    public DrumstickSubsystem() {
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

    public AngularVelocity setVelocityRPM(AngularVelocity rpm) {
        m_velocityLeader.setControl(velocityRequest.withVelocity(rpm.in(RotationsPerSecond)).withEnableFOC(true));
        targetRPM = rpm;
        return targetRPM;
    }

    // This command will run the shooter until it reaches the target speed, then return.
    public Command runToSpeed(AngularVelocity rpm) {
        return Commands.run(() -> setVelocityRPM(rpm), this).until(() -> atSpeed(rpm));
    }

    @Logged(name = "Drumstick getVelocityRPM")
    public AngularVelocity getVelocityRPM() {
        return RPM.of(m_velocityLeader.getVelocity().getValueAsDouble() * 60);
    }

    public boolean atSpeed(AngularVelocity target) {
        atTarget = getVelocityRPM().isNear(target, tolerance);
        return getVelocityRPM().isNear(target, tolerance);
    }

    public Command stopShooter() {
        return Commands.runOnce(() -> m_velocityLeader.stopMotor());
    }

    public void stopMotor() {
        m_velocityLeader.stopMotor();
    }

    @Override
    public void periodic() {
        // if (Robot.count % 10 == 0) {
        //     SmartDashboard.putNumber("Drum Vel", getVelocityRPM());
        //     SmartDashboard.putBoolean("Drum At Speed", atSpeed(targetRPM));
        // }
        if (Robot.count % 100 == 0 && getVelocityRPM().abs(RPM) != 0) {
            logf("Drum Velocity:%.2f, At Speed:%b Target:%.2f",
                    getVelocityRPM(), atSpeed(targetRPM), targetRPM);
        }

    }
}

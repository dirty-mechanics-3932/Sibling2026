package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionMotorSubsystem extends SubsystemBase {

    public final SparkMax m_positionMotor;
    private final SparkRelativeEncoder m_positionEncoder;
    private final SparkClosedLoopController m_positionPID;

    public PositionMotorSubsystem() {
        m_positionMotor = new SparkMax(21, MotorType.kBrushless); // Remember to set the motor IDs
        m_positionEncoder = (SparkRelativeEncoder) m_positionMotor.getEncoder();
        m_positionPID = m_positionMotor.getClosedLoopController();

        SparkMaxConfig positionConfig = new SparkMaxConfig();
        positionConfig.closedLoop
            .pid(0.1, 0.0, 0.0);

        m_positionMotor.configure(positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPositionSetpoint(double value) {
        m_positionPID.setSetpoint(value, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getPositionMotor() {
        return m_positionEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position Motor", getPositionMotor());
    }
}

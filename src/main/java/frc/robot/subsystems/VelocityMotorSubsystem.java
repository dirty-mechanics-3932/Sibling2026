package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityMotorSubsystem extends SubsystemBase {
    
    public final SparkMax m_velocityMotor;
    private final SparkRelativeEncoder m_velocityEncoder;
    private final SparkClosedLoopController m_velocityPID;

    public VelocityMotorSubsystem() {
        m_velocityMotor = new SparkMax(0, MotorType.kBrushless); // Remember to set the motor IDs
        m_velocityEncoder = (SparkRelativeEncoder) m_velocityMotor.getEncoder();
        m_velocityPID = m_velocityMotor.getClosedLoopController();

        SparkMaxConfig velocityConfig = new SparkMaxConfig();
        velocityConfig.closedLoop
            .pid(0.01, 0.0, 0.0)
            .velocityFF(0.01); // I haven't figured out how to do this with ClosedLoopConfig yet

        m_velocityMotor.configure(velocityConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVelocitySetpoint(double value) {
        m_velocityPID.setSetpoint(value, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public double getVelocityMotor() {
        return m_velocityEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Velocity Motor", getVelocityMotor());
    }
}

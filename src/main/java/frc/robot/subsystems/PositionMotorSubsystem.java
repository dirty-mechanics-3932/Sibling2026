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
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;

public class PositionMotorSubsystem extends SubsystemBase {

    public final SparkMax m_positionMotor;
    private final SparkRelativeEncoder m_positionEncoder;
    private final SparkClosedLoopController m_positionPID;
    private final SparkMaxConfig positionConfig;
    double gearRatio = 1.0;

    public PositionMotorSubsystem() {
        m_positionMotor = new SparkMax(21, MotorType.kBrushless); // Remember to set the motor IDs
        m_positionEncoder = (SparkRelativeEncoder) m_positionMotor.getEncoder();
        m_positionPID = m_positionMotor.getClosedLoopController();
        positionConfig = new SparkMaxConfig();

        positionConfig.closedLoop.pid(1, 0.0, 0); //feedForward.kS(.1).kV(0.002).kA(0);    
        positionConfig.closedLoop.maxMotion.maxAcceleration(3000)
        .cruiseVelocity(6000)
        .allowedProfileError(.05);
        // positionConfig.closedLoop.feedForward
        //     .kS(0.2)
        //     .kV(0.0)
        //     .kA(0.0)
        //     .kCos(0.8)
        //     .kCosRatio(1.0/gearRatio);
        m_positionMotor.configure(positionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   
    public void setPositionSetpoint(double value) {
        m_positionPID.setSetpoint(value, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    public double getPositionMotor() {
        return m_positionEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position Motor", getPositionMotor());
    }
}

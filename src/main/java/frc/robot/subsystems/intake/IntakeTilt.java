package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class IntakeTilt extends SubsystemBase {

    public final TalonFX m_positionMotor;
    private final TalonFXConfiguration posConfiguration;
    
    double gearRatio = 1.0;

    public IntakeTilt() {
        m_positionMotor = new TalonFX(21); // Remember to set the motor IDs
        posConfiguration = new TalonFXConfiguration();

        posConfiguration.Slot0.kP = 0.01;
        posConfiguration.Slot0.kI = 0.00;
        posConfiguration.Slot0.kD = 0.00;
        posConfiguration.Slot0.kS = 0.25;
        posConfiguration.Slot0.kV = 0.12;
        
        m_positionMotor.getConfigurator().apply(posConfiguration);

    }

   
    public void setPositionSetpoint(double value) {
  MotionMagicVoltage request = new MotionMagicVoltage(value);
  m_positionMotor.setControl(request);
    }

    public double getPositionMotor() {
        return m_positionMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position Motor", getPositionMotor());
    }
}

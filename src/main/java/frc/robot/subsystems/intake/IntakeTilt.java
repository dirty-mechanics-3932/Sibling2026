package frc.robot.subsystems.intake;


import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTilt extends SubsystemBase {

    public final TalonFX m_positionMotor;
    private final TalonFXConfiguration posConfiguration;
    private final PositionVoltage positionVoltage;
    private final DigitalInput limitSwitch = new DigitalInput(0);
    
    double gearRatio = 1.0;

    public IntakeTilt() {
        m_positionMotor = new TalonFX(21); // Remember to set the motor IDs
        posConfiguration = new TalonFXConfiguration();
        positionVoltage = new PositionVoltage(0);

        posConfiguration.Slot0.kP = 4.5;
        posConfiguration.Slot0.kI = 0.00;
        posConfiguration.Slot0.kD = 0.00;
        posConfiguration.Slot0.kS = 0.25;
        posConfiguration.Slot0.kV = 0.12;
        
        m_positionMotor.getConfigurator().apply(posConfiguration);
    }

    public void zeroEncoder(){
        m_positionMotor.setPosition(0);
    }
    public void setPositionSetpoint(double value) {
        m_positionMotor.setControl(positionVoltage.withPosition(value).withEnableFOC(true)); //in rotations
    }

    public double getPositionMotor() {
        return m_positionMotor.getPosition().getValueAsDouble();
    }

    public void homeIntake(){
        while (limitSwitch.get()) {
         m_positionMotor.set(.1);   
        }
        m_positionMotor.set(0); 
        m_positionMotor.setPosition(0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position Motor", getPositionMotor());
        SmartDashboard.putBoolean("Intake Limit", limitSwitch.get());
    }
}

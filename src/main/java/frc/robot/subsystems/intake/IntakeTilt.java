package frc.robot.subsystems.intake;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTilt extends SubsystemBase {

    public final TalonFX m_intakeTiltMotor; // gear ratio = 52.5
    private final TalonFXConfiguration config;
    private final PositionVoltage positionVoltage;
    private final DigitalInput limitSwitch = new DigitalInput(1);
    
    private double gearRatio = 52.5;
    double motorRotations;
    private double lastPosition;

    public IntakeTilt() {
        m_intakeTiltMotor = new TalonFX(21); // Remember to set the motor IDs
        config = new TalonFXConfiguration();
        positionVoltage = new PositionVoltage(0);

        config.Slot0.kP = 4.5;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
        
        m_intakeTiltMotor.getConfigurator().apply(config);
    }

    public void zeroEncoder(){
        m_intakeTiltMotor.setPosition(0);
    }

    public Command setIntakeTiltSetpoint(double value) {
        return Commands.runOnce(()->m_intakeTiltMotor.setControl(positionVoltage.withPosition(value).withEnableFOC(true))); //in rotations
    }

    public double getPositionMotor() {
        return m_intakeTiltMotor.getPosition().getValueAsDouble();
    }

    public void homeIntake(){
        while (limitSwitch.get()) {
            m_intakeTiltMotor.set(-.1);   
        }
        m_intakeTiltMotor.set(0); 
        zeroEncoder();
    }

    // public Command extendIntake(double degrees) {
    //     lastPosition = degrees;
    //     motorRotations = gearRatio * (degrees/360);
    //     return Commands.runOnce(()->setIntakeTiltSetpoint(motorRotations));
    // }

    // public Command bounceIntake(double degrees) {
    //     lastPosition = degrees;
    //     motorRotations = gearRatio * (degrees/360);
    //     return Commands.runOnce(()->setIntakeTiltSetpoint(motorRotations));
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeTiltPos", getPositionMotor());
        SmartDashboard.putNumber("IntakeTiltDeg", lastPosition);
        SmartDashboard.putBoolean("IntakeLimit", limitSwitch.get());
    }
}

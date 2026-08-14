package frc.robot.subsystems.shooter;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public final TalonFX m_hoodMotor;

    private final CANcoder absoluteEncoder;
    private final TalonFXConfiguration config;
    private final PositionVoltage positionVoltage;
    private final CANcoderConfiguration configEncoder;
    private double targetPosition; 
    private double differenceToTarget;
    private double tolerance = 5; 

    double gearRatio = 1; //Gear ratio is 4.5454 
    
    public HoodSubsystem() {
        m_hoodMotor = new TalonFX(50); // Remember to set the motor IDs
        absoluteEncoder = new CANcoder(51);
        config = new TalonFXConfiguration();
        configEncoder = new CANcoderConfiguration();
        positionVoltage = new PositionVoltage(0);
        
        configEncoder.MagnetSensor.MagnetOffset = 0.563476; 
        config.Slot0.kP = 4.5;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.Feedback.SensorToMechanismRatio = gearRatio; 
        

        m_hoodMotor.getConfigurator().apply(config);
        absoluteEncoder.getConfigurator().apply(configEncoder);

    }

    public double getHoodPosition() {
        // return absoluteEncoder.get();
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public Command setHoodPosition(double value) {
        return Commands.run(() -> setPositionWithEncoder(value),this).until(()->atPosition(value));
    }

    public void zeroEncoder(){
        absoluteEncoder.setPosition(0);
        m_hoodMotor.setPosition(0);
    }

    public void setPositionWithEncoder(double value){
        targetPosition = value;
        m_hoodMotor.setControl(positionVoltage.withPosition(value/360).withEnableFOC(true));
    }

    public Command stop() {
       return Commands.runOnce(()->m_hoodMotor.stopMotor());
    }

    public boolean atPosition(double target){
        if (Math.abs(getHoodPosition() - target) <= tolerance ){
            return true;
        }
        return false; 
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getHoodPosition());
    }
}

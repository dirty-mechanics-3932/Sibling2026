package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public final TalonFX m_hoodMotor;

    private final DutyCycleEncoder absoluteEncoder;
    private final TalonFXConfiguration config;
    private final PositionVoltage positionVoltage;

    double gearRatio = 1.0; //12 to  40 + 22 to 30
    double targetPosition;
    
    public HoodSubsystem() {
        m_hoodMotor = new TalonFX(50); // Remember to set the motor IDs

        absoluteEncoder = new DutyCycleEncoder(0);
        config = new TalonFXConfiguration();
        positionVoltage = new PositionVoltage(0);
        
        config.Slot0.kP = 4.5;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;

        m_hoodMotor.getConfigurator().apply(config);

        // pidController = new PIDController(0.1, 0, 0);
        // pidController.enableContinuousInput(0.0, 1.0);
    }

    public double getHoodPosition() {
        // return absoluteEncoder.get();
        return m_hoodMotor.getPosition().getValueAsDouble();
    }

    public Command setHoodPosition(double value) {
        targetPosition = value;
        return Commands.runOnce(() -> m_hoodMotor.setControl(positionVoltage.withPosition(value).withEnableFOC(true)));
    }

    public boolean atTargetPosition() {
        double tolerance = 1.0;
        return Math.abs(getHoodPosition() - targetPosition) < tolerance;
    }

    public void zeroEncoder(){
        m_hoodMotor.setPosition(0);
    }

    public Command stop() {
       return Commands.runOnce(()->m_hoodMotor.stopMotor());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getHoodPosition());
    }
}

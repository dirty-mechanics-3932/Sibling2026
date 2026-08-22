package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HotDog {
    public final  TalonFX m_velocityMotor;
    private final TalonFXConfiguration configs = new TalonFXConfiguration();
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); 

    public HotDog() {
        m_velocityMotor = new TalonFX(60); // Remember to set the motor IDs
        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        m_velocityMotor.getConfigurator().apply(configs);
    }

    public Command setVelocitySetpoint(double value) {
        return Commands.runOnce(()->m_velocityMotor.setControl(velocityRequest.withVelocity(value/60).withEnableFOC(true)));
    }

    public double getVelocityMotor() {
        return m_velocityMotor.getVelocity().getValueAsDouble();
    }

    public Command stopHotDog() {
        return Commands.runOnce(()->m_velocityMotor.stopMotor());
    }
}

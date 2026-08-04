package frc.robot.subsystems.hotDog;

import java.time.Period;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HotDog {
    public final  TalonFX m_velocityMotor;
    private final TalonFXConfiguration configs = new TalonFXConfiguration();
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); 

    public HotDog() {
        m_velocityMotor = new TalonFX(0); // Remember to set the motor IDs
        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;

        m_velocityMotor.getConfigurator().apply(configs);
    }

    public void setVelocitySetpoint(double value) {
        m_velocityMotor.setControl(velocityRequest.withVelocity(50).withEnableFOC(true));
    }

    public double getVelocityMotor() {
        return m_velocityMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        m_velocityMotor.stopMotor();
    }
}

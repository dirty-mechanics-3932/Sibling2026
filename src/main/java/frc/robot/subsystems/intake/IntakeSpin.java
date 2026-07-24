package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSpin extends SubsystemBase {

    public final TalonFX m_velocityMotor;
    private final TalonFXConfiguration configs;
    private final VelocityVoltage velocityRequest;

    public IntakeSpin() {
        m_velocityMotor = new TalonFX(20); // Remember to set the motor IDs
        configs = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;

        // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_velocityMotor.getConfigurator().apply(configs);
    }

    public void setVelocitySetpoint(double value) {
        m_velocityMotor.setControl(velocityRequest.withVelocity(value).withEnableFOC(true));
    }

    public double getVelocityMotor() {
        return m_velocityMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        m_velocityMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Velocity Motor", getVelocityMotor());
    }
}

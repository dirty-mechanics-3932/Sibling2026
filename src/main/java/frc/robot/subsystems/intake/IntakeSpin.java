package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSpin extends SubsystemBase {

    public final TalonFX m_intakeSpinMotor;
    private final TalonFXConfiguration config;
    private final VelocityVoltage velocityRequest;

    public IntakeSpin() {
        m_intakeSpinMotor = new TalonFX(20); // Remember to set the motor IDs
        config = new TalonFXConfiguration();
        velocityRequest = new VelocityVoltage(0);

        config.Slot0.kP = 0.01;
        config.Slot0.kI = 0.00;
        config.Slot0.kD = 0.00;
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        // configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_intakeSpinMotor.getConfigurator().apply(config);
    }

    public void setVelocitySetpoint(AngularVelocity valueRPM) {
        double rps = valueRPM.in(RotationsPerSecond);
        m_intakeSpinMotor.setControl(velocityRequest.withVelocity(rps).withEnableFOC(true));
    }

    public Command intakeSpin(AngularVelocity valueRPM) {
        return Commands.runOnce(()->setVelocitySetpoint(valueRPM));
    }

    public double getVelocity() {
        return m_intakeSpinMotor.getVelocity().getValueAsDouble() * 60;
    }

    public Command stopIntake() {
        return Commands.runOnce(()->m_intakeSpinMotor.stopMotor());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeSpinVel", getVelocity());
    }
}

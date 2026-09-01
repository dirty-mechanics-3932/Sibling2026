package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HotDog extends SubsystemBase {
    public final  TalonFX m_velocityMotor;
    private final TalonFXConfiguration configs = new TalonFXConfiguration();
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); 

    @Logged(name = "HotDog setVelocitySetpoint")
    private AngularVelocity targetRPM = RPM.of(0.0);

    public HotDog() {
        m_velocityMotor = new TalonFX(60);
        configs.Slot0.kP = 0.01;
        configs.Slot0.kI = 0.00;
        configs.Slot0.kD = 0.00;
        configs.Slot0.kS = 0.25;
        configs.Slot0.kV = 0.12;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 

        m_velocityMotor.getConfigurator().apply(configs);
    }

    public Command setVelocitySetpointCmd(AngularVelocity valueRPM) {
        return Commands.runOnce(()->m_velocityMotor.setControl(velocityRequest.withVelocity(valueRPM.in(RotationsPerSecond)).withEnableFOC(true)));
    }

    public AngularVelocity setVelocitySetpoint(AngularVelocity valueRPM){
        m_velocityMotor.setControl(velocityRequest.withVelocity(valueRPM.in(RotationsPerSecond)).withEnableFOC(true));  
        targetRPM = valueRPM;
        return valueRPM;
    }

    @Logged(name = "HotDog getVelocityRPM")
    public AngularVelocity getVelocityMotor() {
        return RPM.of(m_velocityMotor.getVelocity().getValueAsDouble() * 60);
    }

    public Command stopHotDog() {
        return Commands.runOnce(()->m_velocityMotor.stopMotor());
    }

    public void stopMotor() {
       m_velocityMotor.stopMotor();
    }
}

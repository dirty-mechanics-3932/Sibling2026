package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public final SparkMax m_hoodMotor;

    private final DutyCycleEncoder absoluteEncoder;
    private final SparkMaxConfig config;
    private final PIDController pidController;

    private double targetPosition = 0.0;
    
    public HoodSubsystem() {
        m_hoodMotor = new SparkMax(50, MotorType.kBrushless); // Remember to set the motor IDs

        absoluteEncoder = new DutyCycleEncoder(0);
        config = new SparkMaxConfig();

        config.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
        config.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        m_hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController = new PIDController(0.1, 0, 0);
        pidController.enableContinuousInput(0.0, 1.0);
    }

    public double getPosition() {
        return absoluteEncoder.get();
    }

    public void setTargetPosition(double value) {
        targetPosition = Math.max(0.0, Math.min(1.0, value));
    }

    public boolean atTargetPosition() {
        return pidController.atSetpoint();
    }

    public void stop() {
        m_hoodMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (absoluteEncoder.isConnected()) {
            double pidOutput = pidController.calculate(getPosition(), targetPosition);
        
            m_hoodMotor.set(pidOutput);

            SmartDashboard.putNumber("Hood Target Position", targetPosition);
        } else {
            stop();
        }
    }
}

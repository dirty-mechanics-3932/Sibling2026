package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.Pose.PositionSubsystem;
import frc.robot.subsystems.intake.IntakeSpin;
import frc.robot.subsystems.intake.IntakeTilt;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.HotDog;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionAndShootCommand extends Command {

    /*
     * Valor values
     * Intake: 3600
     * Hopper (hotdog): 1320
     * Catchup: 1500
     * 
     */
    private final PositionSubsystem m_positionSubsysten;
    private final DrumstickSubsystem m_drumstickSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final HotDog m_hotDog;
    private final CatchupSubsystem m_catchupSubsystem;
    //TODO: Dont delete, requested to add wheel lock when shooting
    private final SwerveSubsystem m_drivebase;
    private final IntakeTilt m_IntakeTilt;
    private final IntakeSpin m_IntakeSpin;

    private AngularVelocity shooterTarget;
    private Angle hoodTarget;
    private Distance distToTarget;
    private boolean atSpeed;
    private boolean finished;
    private int my_count = 0;

    public PositionAndShootCommand(PositionSubsystem position, DrumstickSubsystem drumstick, HoodSubsystem hood,
            HotDog hotDog, CatchupSubsystem catchup, SwerveSubsystem swerve, IntakeTilt intakeTilt,
            IntakeSpin intakeSpin) {
        this.m_positionSubsysten = position;
        this.m_drumstickSubsystem = drumstick;
        this.m_hotDog = hotDog;
        this.m_hoodSubsystem = hood;
        this.m_catchupSubsystem = catchup;
        this.m_drivebase = swerve;
        this.m_IntakeTilt = intakeTilt;
        this.m_IntakeSpin = intakeSpin;

        addRequirements(position, catchup, hood, drumstick);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Spin flywheel
        atSpeed = false;
        shooterTarget = m_positionSubsysten.getTargetShooterRPM();
        hoodTarget = m_positionSubsysten.getHoodPosition();
        distToTarget = Meters.of(m_positionSubsysten.getDistanceV2());
        m_drumstickSubsystem.setVelocityRPM(shooterTarget);
        m_hoodSubsystem.setPositionWithEncoder(hoodTarget);
        logf("Start shoot command, Target RPM: %.3f , Target hood %.3f , distance %.3f", shooterTarget, hoodTarget, distToTarget);
        //m_hotDog.setVelocitySetpoint(3000);
        finished = false;
    }

    @Override
    public void execute() {
        my_count++;
        if (!atSpeed && m_positionSubsysten.readyToShoot(m_drumstickSubsystem, m_hoodSubsystem)) {
            m_catchupSubsystem.setCatchupSetpoint(RPM.of(3000)); //shooterTarget);
            m_hotDog.setVelocitySetpoint(RPM.of(3000));
            my_count = 0;
            atSpeed = true;
        }
        if (atSpeed && my_count >= 13) {
            m_IntakeTilt.moveIntakeTiltDeltaDeg(-70);
            m_IntakeSpin.setVelocitySetpoint(3000);
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        logf("Ended shoot command. drumstick speed %.3f , hood pos %.3f, catchup %.3f , hotdog %.3f",
                m_drumstickSubsystem.getVelocityRPM().in(RPM), m_hoodSubsystem.getHoodPositionInDeg().in(Degrees),
                m_catchupSubsystem.getVelocityRPS().in(RPM), m_hotDog.getVelocityMotor().in(RPM));
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
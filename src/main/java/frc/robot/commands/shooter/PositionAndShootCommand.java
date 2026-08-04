package frc.robot.commands.shooter;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.PositionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionAndShootCommand extends Command {

    private final PositionSubsystem m_positionSubsysten;
    private final DrumstickSubsystem m_drumstickSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final CatchupSubsystem m_catchupSubsystem;
    private final SwerveSubsystem m_drivebase;

    public PositionAndShootCommand(PositionSubsystem position, DrumstickSubsystem drumstick, HoodSubsystem hood, CatchupSubsystem catchup, SwerveSubsystem swerve){
        this.m_positionSubsysten = position;
        this.m_drumstickSubsystem = drumstick;
        this.m_hoodSubsystem = hood;
        this.m_catchupSubsystem = catchup;
        this.m_drivebase = swerve;

        addRequirements(position, catchup, hood, drumstick);
    }

    @Override
    public void execute(){
        // Spin flywheel
        m_drumstickSubsystem.setVelocitySetpoint(m_positionSubsysten.getShooterRPM());

        // Angle hood
        m_hoodSubsystem.setTargetPosition(m_positionSubsysten.getHoodPosition());

        // Aim robot code
        //double targetDirection = m_positionSubsysten.getTargetHeading() - m_drivebase.getHeading().getDegrees() * 0.1;
        //m_drivebase.drive(new Translation2d(), targetDirection, true);

        if(m_positionSubsysten.readyToShoot(m_drumstickSubsystem, m_hoodSubsystem)) {
            if (Robot.showAllLogs && Robot.count % 50 == 5) {
                Boolean atSpeed = Math.abs(m_drumstickSubsystem.getVelocityMotor() - m_positionSubsysten.getShooterRPM()) < 100;
                logf(
                    "Shooter-na req:%.2f act:%.2f ok:%b angle req:%.2f act:%.2f ok:%b hood:%.2f",
                    m_positionSubsysten.getShooterRPM(),
                    m_drumstickSubsystem.getVelocityMotor(),
                    atSpeed,
                    m_positionSubsysten.getTargetHeading(),
                    m_drivebase.getHeading(),
                    m_positionSubsysten.atHeading(),
                    m_hoodSubsystem.getPosition()
                );
            }
            m_catchupSubsystem.setVelocitySetpoint(6000);
        }else{
            m_catchupSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drumstickSubsystem.stop();
        m_catchupSubsystem.stop();
        m_hoodSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
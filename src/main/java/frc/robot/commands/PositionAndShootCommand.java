package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

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

    private final PositionSubsystem m_positionSubsysten;
    private final DrumstickSubsystem m_drumstickSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final HotDog m_hotDog; 
    private final CatchupSubsystem m_catchupSubsystem;
    private final SwerveSubsystem m_drivebase;
    private final IntakeTilt m_IntakeTilt;
    private final IntakeSpin m_IntakeSpin;

    private double shooterTarget;
    private double hoodTarget;
    private double distToTarget; 
    private boolean atSpeed; 
    private boolean finished;
    private int my_count = 0; 

    public PositionAndShootCommand(PositionSubsystem position, DrumstickSubsystem drumstick, HoodSubsystem hood, HotDog hotDog, CatchupSubsystem catchup, SwerveSubsystem swerve, IntakeTilt intakeTilt, IntakeSpin intakeSpin){
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
     shooterTarget = m_positionSubsysten.getShooterRPM();
     hoodTarget = m_positionSubsysten.getHoodPosition(); 
     distToTarget = m_positionSubsysten.getDistanceV2();
     m_drumstickSubsystem.setVelocityRPM(shooterTarget);
     m_hoodSubsystem.setPositionWithEncoder(hoodTarget/360.0); 
     logf("Start shoot command, Target RPM: %.3f , Target hood %.3f , distance %.3f", shooterTarget, hoodTarget, distToTarget);

  }


    @Override
    public void execute(){
        my_count++;
        if (!atSpeed && m_positionSubsysten.readyToShoot(m_drumstickSubsystem, m_hoodSubsystem)){
            m_catchupSubsystem.setCatchupSetpoint(shooterTarget);
            m_hotDog.setVelocitySetpoint(3000); 
            my_count = 0;
            atSpeed = true; 
        }
        if (atSpeed && my_count>=25){
            m_IntakeTilt.moveIntakeTiltDeltaDeg(-70);
            m_IntakeSpin.setVelocitySetpoint(3000);
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted){
        logf("Ended shoot command. drumstick speed %.3f , hood pos %.3f, catchup %.3f , hotdog %.3f", m_drumstickSubsystem.getVelocityRPM(), m_hoodSubsystem.getHoodPositionInDeg(), m_catchupSubsystem.getVelocity(), m_hotDog.getVelocityMotor());
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
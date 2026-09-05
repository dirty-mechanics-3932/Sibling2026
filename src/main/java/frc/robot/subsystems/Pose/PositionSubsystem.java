package frc.robot.subsystems.Pose;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Velocity;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionSubsystem extends SubsystemBase {

    private final SwerveSubsystem m_drivebase;
    @Logged(name = "Drum RPM Tolerance")
    AngularVelocity drumTolerance = RPM.of(50);

    @Logged(name = "Target Heading Tolerance")
    Angle headingTolerance = Degrees.of(2);

    private Pose2d hubPoseBlue = new Pose2d(Meters.of(4.63), Meters.of(4.03), new Rotation2d(Degrees.of(0)));
    private Pose2d hubPoseRed = new Pose2d(Meters.of(11.92), Meters.of(4.03), new Rotation2d(Degrees.of(180)));

    private Pose2d neutralRedRight = new Pose2d(Meters.of(14.42), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
    private Pose2d neutralRedLeft = new Pose2d(Meters.of(14.42), Meters.of(2.01), new Rotation2d(Degrees.of(0)));
    private Pose2d neutralBlueLeft = new Pose2d(Meters.of(2.31), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
    private Pose2d neutralBlueRight = new Pose2d(Meters.of(2.31), Meters.of(2.01), new Rotation2d(Degrees.of(0)));
    @Logged
    private boolean isTargetOverrided = false;
    private Pose2d compensatedPose = new Pose2d(); 

    //This is default blue hub?
    private Pose2d target  = new Pose2d(Meters.of(4.63), Meters.of(4.03), new Rotation2d(Degrees.of(0)));
    @Logged
    private String targetName = "";
    @Logged
    private double distance = 0.0; //Not changing to unit because its used almost exclusivly to read from the double tree map and that would be inefficient
    @Logged
    private Angle targetHeading = Degrees.of(0.0);
    @Logged
    private AngularVelocity targetShooterRPM = RPM.of(0.0);
    @Logged
    private Angle hoodPosition = Degrees.of(0.0);

    private final InterpolatingDoubleTreeMap shootData = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodData = new InterpolatingDoubleTreeMap();

    public PositionSubsystem(SwerveSubsystem swerve) {
        this.m_drivebase = swerve;

        shootData.put(1.0, 2700.0);
        shootData.put(2.0, 3070.0);
        shootData.put(2.5, 3300.0);
        shootData.put(3.0, 3450.0);
        shootData.put(3.5, 3625.0);
        shootData.put(4.0, 3825.0);
        shootData.put(5.0, 4650.0);
        shootData.put(6.0, 4700.0);
        shootData.put(7.0, 5000.0);
        shootData.put(22.0, 4500.0);

        hoodData.put(1.0, 0.0);
        hoodData.put(2.0, 0.0);
        hoodData.put(3.0, 9.0);
        hoodData.put(3.5, 8.0);
        hoodData.put(4.0, 8.0);
        hoodData.put(5.0, 15.0);
        hoodData.put(6.0, 15.0);
        hoodData.put(7.0, 15.0);
        hoodData.put(20.0, 15.0);
        logShootTable();
    }

    public void logShootTable() {
        // for (distance = 0; distance < 20; distance += 2) {
        //     logf("**** Dist:%.2f shoot:%.2f hood:%.2f", distance, getShooterRPM(), getHoodPosition());
        // }
    }

    @Override
    public void periodic() {
        updateTarget();
        //if conditional is impossible since we now default target to one of the hubs
        if (target == null)
            return;

        // If this doesn't work just replace 'compensatedPose' with
        // 'm_drivebase.getPose()'
        compensatedPose = m_drivebase.getPose();// compensatedPose();
        distance = compensatedPose.getTranslation().getDistance(target.getTranslation());
        targetShooterRPM = RPM.of(shootData.get(distance));
        hoodPosition = Degrees.of(hoodData.get(distance));
        targetHeading = Degrees.of(m_drivebase.getHeadingToPose(target).getDegrees());
        // if (Robot.count % 10 == 5) {
        //     SmartDashboard.putString("Target Name", targetName);
        //     SmartDashboard.putNumber("Dist to Target", round2(distance));
        //     SmartDashboard.putNumber("Drum Traget", Math.round(shooterRPM));
        //     SmartDashboard.putNumber("Heading to Target", targetHeading);
        //     SmartDashboard.putNumber("Hood Target Pos", round2(hoodPosition));
        // }
    }

    // An attempt to compensate for motion so shooting while moving works, does not
    // currently compensate for rotation
    private Pose2d compensatedPose() {
        Pose2d pose = m_drivebase.getPose();
        ChassisSpeeds speeds = m_drivebase.getFieldVelocity();
        double latency = 0.25;

        return new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * latency,
                pose.getY() + speeds.vyMetersPerSecond * latency,
                pose.getRotation());
    }

    private void updateTarget() {
        if (Robot.isAllianceBlue()) {
            target = hubPoseBlue;
            targetName = "hubPoseBlue";
            if (isInFeedingZone()) {
                Pose2d pose = m_drivebase.getPose();
                if (!isTargetOverrided) {
                    if (pose.getTranslation().getY() < hubPoseBlue.getTranslation().getY()) {
                        target = neutralBlueRight;
                        targetName = "neutralBlueRight";
                    } else if (pose.getTranslation().getY() > hubPoseBlue.getTranslation().getY()) {
                        target = neutralBlueLeft;
                        targetName = "neutralBlueLeft";
                    }
                } else {
                    if (pose.getTranslation().getY() < hubPoseBlue.getTranslation().getY()) {
                        target = neutralBlueLeft;
                        targetName = "neutralBlueLeft";
                    } else if (pose.getTranslation().getY() > hubPoseBlue.getTranslation().getY()) {
                        target = neutralBlueRight;
                        targetName = "neutralBlueRight";
                    }
                }
            }
        } else {
            target = hubPoseRed;
            targetName = "hubPoseRed";
            if (isInFeedingZone()) {
                Pose2d pose = m_drivebase.getPose();
                if (!isTargetOverrided) {
                    if (pose.getTranslation().getY() < hubPoseRed.getTranslation().getY()) {
                        target = neutralRedLeft;
                        targetName = "neutralRedLeft";
                    } else if (pose.getTranslation().getY() > hubPoseRed.getTranslation().getY()) {
                        target = neutralRedRight;
                        targetName = "neutralRedRight";
                    }
                } else {
                    if (pose.getTranslation().getY() < hubPoseRed.getTranslation().getY()) {
                        target = neutralRedRight;
                        targetName = "neutralRedRight";
                    } else if (pose.getTranslation().getY() > hubPoseRed.getTranslation().getY()) {
                        target = neutralRedLeft;
                        targetName = "neutralRedLeft";
                    }
                }
            }
        }
    }

    @Logged(name = "getShooterRPM")
    public AngularVelocity getTargetShooterRPM() {
        return RPM.of(shootData.get(getDistanceV2()));
    }

    @Logged(name = "getDistance")
    public Distance getDistance(){
        return Meters.of(distance); 
    }

    @Logged(name = "getDistanceV2")
    public double getDistanceV2(){
        return compensatedPose.getTranslation().getDistance(target.getTranslation());
    }

    @Logged(name = "getHoodPos Pose Subsystem")
    public Angle getHoodPosition() {
        return Degrees.of(hoodData.get(getDistanceV2()));
    }

    @Logged(name = "Target Heading Method")
    public Angle getTargetHeading() {
        return targetHeading;
    }

    public Pose2d getTarget() {
        return target;
    }

    @Logged(name = "In Feeding Zone") //isInNeutralZone doesnt work anymore as a name because it applpies to op alliance zone as well
    public boolean isInFeedingZone() {
        if (target == null)
            return false;
        Pose2d pose = m_drivebase.getPose();
        if (Robot.isAllianceBlue()) {
            return pose.getTranslation().getX() > target.getTranslation().getX();
        } else {
            return pose.getTranslation().getX() < target.getTranslation().getX();
        }
    }

    // public Command notOverrided() {
    //     return new InstantCommand(() -> isTargetOverrided = false);
    // }

    // public Command targetOverride() {
    //     return new InstantCommand(() -> isTargetOverrided = true);
    // }

    public boolean atHeading() {
        return Degrees.of(m_drivebase.getHeading().getDegrees()).isNear(targetHeading, headingTolerance);
    }

    public boolean readyToShoot(DrumstickSubsystem drumstick, HoodSubsystem hood) {
        return targetShooterRPM.gte(drumTolerance) && drumstick.getVelocityRPM().isNear(targetShooterRPM, drumTolerance);
    }
}
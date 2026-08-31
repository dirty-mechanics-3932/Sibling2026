package frc.robot.subsystems.Pose;

import static frc.robot.utilities.Util.round2;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionSubsystem extends SubsystemBase {

    private final SwerveSubsystem m_drivebase;
    double tolerance = 100;

    private Pose2d hubPoseBlue = new Pose2d(Meters.of(4.63), Meters.of(4.03), new Rotation2d(Degrees.of(0)));
    private Pose2d hubPoseRed = new Pose2d(Meters.of(11.92), Meters.of(4.03), new Rotation2d(Degrees.of(180)));

    private Pose2d neutralRedRight = new Pose2d(Meters.of(14.42), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
    private Pose2d neutralRedLeft = new Pose2d(Meters.of(14.42), Meters.of(2.01), new Rotation2d(Degrees.of(0)));
    private Pose2d neutralBlueLeft = new Pose2d(Meters.of(2.31), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
    private Pose2d neutralBlueRight = new Pose2d(Meters.of(2.31), Meters.of(2.01), new Rotation2d(Degrees.of(0)));

    private boolean isTargetOverrided = false;
    private Pose2d compensatedPose = new Pose2d(); 

    private Pose2d target  = new Pose2d(Meters.of(4.63), Meters.of(4.03), new Rotation2d(Degrees.of(0)));
    private String targetName = "";

    private double distance;
    private double targetHeading;
    private double shooterRPM;
    private double hoodPosition;

    private final InterpolatingDoubleTreeMap shootData = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodData = new InterpolatingDoubleTreeMap();

    public PositionSubsystem(SwerveSubsystem swerve) {
        this.m_drivebase = swerve;

        shootData.put(1.0, 2700.0);
        shootData.put(2.0, 3070.0);
        shootData.put(3.0, 3500.0);
        shootData.put(4.0, 3600.0);
        shootData.put(5.0, 3800.0);
        shootData.put(6.0, 4000.0);
        shootData.put(7.0, 4200.0);
        // shootData.put(8.0,4800.0);
        // shootData.put(9.0,4915.0);
        // shootData.put(10.0,6000.0);
        // shootData.put(20.0,6000.0);
        shootData.put(22.0, 4500.0);

        hoodData.put(1.0, 0.0);
        hoodData.put(2.0, 0.0);
        hoodData.put(3.0, 9.0);
        hoodData.put(4.0, 12.0);
        hoodData.put(5.0, 14.0);
        hoodData.put(6.0, 15.0);
        hoodData.put(7.0, 16.0);
        hoodData.put(20.0, 20.0);
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
        if (target == null)
            return;

        // If this doesn't work just replace 'compensatedPose' with
        // 'm_drivebase.getPose()'
        compensatedPose = m_drivebase.getPose();// compensatedPose();
        distance = compensatedPose.getTranslation().getDistance(target.getTranslation());
        shooterRPM = shootData.get(distance);
        hoodPosition = hoodData.get(distance);
        targetHeading = m_drivebase.getHeadingToPose(target).getDegrees();
        if (Robot.count % 10 == 5) {
            SmartDashboard.putString("Target Name", targetName);
            SmartDashboard.putNumber("Dist to Target", round2(distance));
            SmartDashboard.putNumber("Drum Traget", Math.round(shooterRPM));
            SmartDashboard.putNumber("Heading to Target", targetHeading);
            SmartDashboard.putNumber("Hood Target Pos", round2(hoodPosition));
        }
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
            if (isInNeutralZone()) {
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
            if (isInNeutralZone()) {
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

    public double getShooterRPM() {
        return shootData.get(getDistanceV2());
    }

    public double getDistance(){
        return distance; 
    }

    public double getDistanceV2(){
        return compensatedPose.getTranslation().getDistance(target.getTranslation());
    }
    public double getHoodPosition() {
        return hoodData.get(getDistanceV2());
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public Pose2d getTarget() {
        return target;
    }

    public boolean isInNeutralZone() {
        if (target == null)
            return false;
        Pose2d pose = m_drivebase.getPose();
        if (Robot.isAllianceBlue()) {
            return pose.getTranslation().getX() > target.getTranslation().getX();
        } else {
            return pose.getTranslation().getX() < target.getTranslation().getX();
        }
    }

    public Command notOverrided() {
        return new InstantCommand(() -> isTargetOverrided = false);
    }

    public Command targetOverride() {
        return new InstantCommand(() -> isTargetOverrided = true);
    }

    public boolean atHeading() {
        double tolerance = 2.0;
        return Math.abs(m_drivebase.getHeading().minus(Rotation2d.fromDegrees(targetHeading)).getDegrees()) < tolerance;
    }

    public boolean readyToShoot(DrumstickSubsystem drumstick, HoodSubsystem hood) {
        return shooterRPM > 0 && Math.abs(drumstick.getVelocityRPM() - shooterRPM) <= tolerance;
    }
}
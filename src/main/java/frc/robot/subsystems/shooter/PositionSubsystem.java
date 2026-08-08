package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.FieldConstants;

public class PositionSubsystem extends SubsystemBase {

    private final SwerveSubsystem m_drivebase;

    private boolean isTargetOverrided = false;
    
    private Pose2d target;
    private String targetName = "";

    private double distance;
    private double targetHeadingDegrees;
    private double shooterRPM;
    private double hoodPosition;

    // For the data tables, the reverse data is for when the robot is facing backwards
    private final InterpolatingDoubleTreeMap shootData = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shootReverseData = new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap hoodData = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodReverseData = new InterpolatingDoubleTreeMap();

    public PositionSubsystem(SwerveSubsystem swerve){
        this.m_drivebase = swerve;

        shootData.put(1.0,2500.0);
        shootData.put(2.0,2600.0);
        shootData.put(3.0,2850.0);
        shootData.put(4.0,3000.0);
        shootData.put(5.0,3440.0);
        shootData.put(6.0,3405.0);
        shootData.put(7.0,4700.0);
        shootData.put(8.0,4800.0);
        shootData.put(9.0,4915.0);
        shootData.put(10.0,6000.0);
        shootData.put(20.0,6000.0);

        shootReverseData.put(1.0,2500.0);
        shootReverseData.put(2.0,2600.0);
        shootReverseData.put(3.0,2850.0);
        shootReverseData.put(4.0,3000.0);
        shootReverseData.put(5.0,3440.0);
        shootReverseData.put(6.0,3405.0);
        shootReverseData.put(7.0,4700.0);
        shootReverseData.put(8.0,4800.0);
        shootReverseData.put(9.0,4915.0);
        shootReverseData.put(10.0,6000.0);
        shootReverseData.put(20.0,6000.0);

        hoodData.put(1.0, .23);

        hoodReverseData.put(1.0, .23);
    }

    @Override
    public void periodic(){
        updateTarget();
        if (target == null) return;

        // Pose2d compensatedPose = compensatedPose();
        Pose2d pose = m_drivebase.getPose();
        distance = pose.getTranslation().getDistance(target.getTranslation());
        shooterRPM = m_drivebase.shootBackward(() -> target) ? shootData.get(distance) : shootReverseData.get(distance);
        hoodPosition = m_drivebase.shootBackward(() -> target) ? hoodData.get(distance) : hoodReverseData.get(distance);
        targetHeadingDegrees = target.getTranslation().minus(pose.getTranslation()).getAngle().getDegrees();

        SmartDashboard.putString("Target Name", targetName);
    }

    // private Pose2d compensatedPose(){
    //     Pose2d pose = m_drivebase.getPose();
    //     ChassisSpeeds speeds = m_drivebase.getFieldVelocity();
    //     double latency = 0.25;

    //     return new Pose2d(
    //         pose.getX() + speeds.vxMetersPerSecond * latency,
    //         pose.getY() + speeds.vyMetersPerSecond * latency,
    //         pose.getRotation()
    //     );
    // }

    // public boolean closerToForward() {
    //     Pose2d pose = m_drivebase.getPose();
    //     Rotation2d robotHeading = pose.getRotation();
    //     Rotation2d targetHeading = Rotation2d.fromDegrees(targetHeadingDegrees);
    //     double angleDifference = Math.abs(targetHeading.minus(robotHeading).getDegrees());
    //     return angleDifference <= 90;
    // }

    private void updateTarget() {
        Pose2d pose = m_drivebase.getPose();
        double robotTranslationY = pose.getTranslation().getY();
        double hubBlueTranslationY = FieldConstants.HUB_POSE_BLUE.getTranslation().getY();
        double hubRedTranslationY = FieldConstants.HUB_POSE_RED.getTranslation().getY();
        if(Robot.isAllianceBlue()) {
            target = FieldConstants.HUB_POSE_BLUE;
            targetName = "hubPoseBlue";
            if (isInNeutralZone()) {
                if (!isTargetOverrided) {
                    if (robotTranslationY < hubBlueTranslationY) {
                        target = FieldConstants.NEUTRAL_BLUE_RIGHT;
                        targetName = "neutralBlueRight";
                    } else if (robotTranslationY > hubBlueTranslationY) {
                        target = FieldConstants.NEUTRAL_BLUE_LEFT;
                        targetName = "neutralBlueLeft";
                    }
                } else {
                    if (robotTranslationY < hubBlueTranslationY) {
                        target = FieldConstants.NEUTRAL_BLUE_LEFT;
                        targetName = "neutralBlueLeft";
                    } else if (robotTranslationY > hubBlueTranslationY) {
                        target = FieldConstants.NEUTRAL_BLUE_RIGHT;
                        targetName = "neutralBlueRight";
                    }
                }
            }
        } else {
            target = FieldConstants.HUB_POSE_RED;
            targetName = "hubPoseRed";
            if (isInNeutralZone()) {
                if (!isTargetOverrided) {
                    if (robotTranslationY < hubRedTranslationY) {
                        target = FieldConstants.NEUTRAL_RED_LEFT;
                        targetName = "neutralRedLeft";
                    } else if (robotTranslationY > hubRedTranslationY) {
                        target = FieldConstants.NEUTRAL_RED_RIGHT;
                        targetName = "neutralRedRight";
                    }
                } else {
                    if (robotTranslationY < hubRedTranslationY) {
                        target = FieldConstants.NEUTRAL_RED_RIGHT;
                        targetName = "neutralRedRight";
                    } else if (robotTranslationY > hubRedTranslationY) {
                        target = FieldConstants.NEUTRAL_RED_LEFT;
                        targetName = "neutralRedLeft";
                    }
                }
            }
        }
    }

    public double getShooterRPM() {
        return shooterRPM;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getTargetHeadingDegrees() {
        return targetHeadingDegrees;
    }

    public Pose2d getTarget() {
        return target;
    }

    public String getTargetName() {
        return targetName;
    }

    public boolean isInNeutralZone() {
        if (target == null) return false;
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
        return Math.abs(m_drivebase.getHeading().minus(Rotation2d.fromDegrees(targetHeadingDegrees)).getDegrees()) < tolerance;
    }

    public boolean readyToShoot(DrumstickSubsystem drumstick, HoodSubsystem hood) {
        return atHeading() && shooterRPM > 0 && hood.atTargetPosition() && Math.abs(drumstick.getVelocity() - shooterRPM) < 100;
    }
}
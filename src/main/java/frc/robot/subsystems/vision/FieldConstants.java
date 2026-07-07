package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {

  // Hub poses for each alliance. These are the same for both alliances, but the coordinates are mirrored.
  public  static final Pose2d HUB_POSE_BLUE =
      new Pose2d(Meters.of(4.63), Meters.of(4.03), new Rotation2d(Degrees.of(0)));
  public  static final Pose2d HUB_POSE_RED =
      new Pose2d(Meters.of(11.92), Meters.of(4.03), new Rotation2d(Degrees.of(180)));

  // Neutral poses
  public  static final Pose2d NEUTRAL_RED_RIGHT =
      new Pose2d(Meters.of(14.42), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
  public  static final Pose2d NEUTRAL_RED_LEFT =
      new Pose2d(Meters.of(14.42), Meters.of(2.01), new Rotation2d(Degrees.of(0)));
  public  static final Pose2d NEUTRAL_BLUE_LEFT =
      new Pose2d(Meters.of(2.31), Meters.of(6.01), new Rotation2d(Degrees.of(180)));
  public  static final Pose2d NEUTRAL_BLUE_RIGHT =
      new Pose2d(Meters.of(2.31), Meters.of(2.01), new Rotation2d(Degrees.of(0)));
    
}

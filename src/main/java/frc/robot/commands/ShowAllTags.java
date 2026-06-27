package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Optional;

public class ShowAllTags extends Command {
  /** Creates a new ReplaceMeCommand. */
  int tag = 1; // Starting tag

  int count; // Used to delay so user can see the updated tag
  AprilTagFieldLayout fieldLayout;

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  // StructArrayPublisher<Pose2d> arrayPublisher =
  //    NetworkTableInstance.getDefault().getStructArrayTopic("Array", Pose2d.struct).publish();

  // tag -- > 0 drive to that tag,
  // 0 - drive to visible tag,
  // < 0 -- move image of robot to all of the FRC defined april tags
  public ShowAllTags(SwerveSubsystem swerve) {
    addRequirements(swerve);
    try {
      // Load the default field layout (2025 FRC field)
      fieldLayout =
          AprilTagFieldLayout.loadField(
              AprilTagFields.k2026RebuiltWelded); // Or kDefaultField for the
      // current season
    } catch (Exception e) {
      // Handle error: e.g., log or use a fallback layout
      fieldLayout = null;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tag = 1;
    count = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (count > 0) {
      // delay to allow user to see tag
      count--;
      return;
    }
    count = 20;
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tag);
    Pose3d tagPose = tagPoseOpt.get();
    Pose2d p2 = tagPose.toPose2d();
    Transform2d backTransform =
        new Transform2d(
            new Translation2d(.7, 0.0), // -0.5m in robot's X, 0 in Y
            new edu.wpi.first.math.geometry.Rotation2d(Math.PI) // 180 degreeo rotation
            );
    Pose2d newPose = p2.transformBy(backTransform);
    publisher.set(newPose);
    logf("Tag %d pose:%s ", tag, newPose.toString());
    // arrayPublisher.set(new Pose2d[] {newPose});
    tag++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Pose2d newPose = new Pose2d(1.2, 1.2, new edu.wpi.first.math.geometry.Rotation2d(0.0));
    publisher.set(newPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tag > 32;
  }
}

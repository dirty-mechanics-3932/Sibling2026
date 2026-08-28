package frc.robot.subsystems.Pose;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightResults;

import java.util.Optional;

/** FRC robot code to interface with Limelight and display pose data. */
public class VisionSubsystemV2 extends SubsystemBase {
  private String m_systemName = "";
  // Initial Limelight camera does not need a name for lookup
  private String m_cameraName = "limelight-rear";
  private String m_shortName = "R_";
  private int cyclecount = 0;
  private int tagID = 0;
  private boolean tv = true;
  private Pose2d pose;

  private final frc.robot.subsystems.swervedrive.SwerveSubsystem m_swerveDrive; // Your SwerveDrive class

  private static final AprilTagFieldLayout m_fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault()
          .getStructTopic(m_shortName + ":MyPose", Pose2d.struct)
          .publish();

  public VisionSubsystemV2(SwerveSubsystem swerveDrive, String cameraName, String shortName) {
    try {
      m_swerveDrive = swerveDrive;
      m_cameraName = cameraName;
      m_systemName = cameraName;
      m_shortName = shortName;
      LimelightHelpers.SetIMUMode(m_cameraName, 3);
      // limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      setName(m_systemName);
    } catch (Exception e) {
      throw new RuntimeException(
          "Failed to load AprilTag field layout for:" + m_cameraName + "-" + e.getMessage());
    }
  }

  public static class NoValidPoseException extends Exception {
    public NoValidPoseException(String msg) {
      super("Pose unavailable: " + msg);
    }
  }

  @Override
  public void periodic() {
    try {
      tv = LimelightHelpers.getTV(m_cameraName);
      SmartDashboard.putBoolean(m_shortName + " Tv ", tv);
      if (tv) {
        updateSmartDashboard();
        updateSwerveOdometry();
      }
    } catch (Exception e) {
      logf("Error in VisionSubsystem periodic: %s", e.getMessage());
    }
  }

  String visionResult = "";

  public String getVisionResult() {
    if (tv) {
      visionResult =
          String.format(
              "%s Tag:%d X:%.3f Y:%.3f R:%.2f",
              m_shortName,
              tagID,
              pose.getMeasureX().in(Meters),
              pose.getMeasureY().in(Meters),
              pose.getRotation().getDegrees());
      return visionResult;
    } else {
      return m_shortName + " Has no valid vision result";
    }
  }
  public void switchPipeline(int num){
    LimelightHelpers.setPipelineIndex(m_cameraName, num);
  }

  public double getPipeline(){
    LimelightResults results = LimelightHelpers.getLatestResults(m_cameraName);
    return results.pipelineID; 
  }

  private void updateSwerveOdometry() throws NoValidPoseException {
    pose = getRobotPose().orElseThrow();
    // Get the timestamp of the measurement from the Limelight (latency)
    // 'tstamp' is the pipeline latency plus capture latency
    double latencySeconds =
        LimelightHelpers.getLimelightNTTable(m_cameraName).getEntry("tstamp").getDouble(0.0)
            / 1000.0;
    double currentTime = Timer.getFPGATimestamp();
    double imageCaptureTime = currentTime - latencySeconds;

    // Add the vision measurement to the SwerveDrive pose estimator
    // You can optionally tune standard deviations here; see YAGSL docs for "Tuning
    // out Drift"
    int tagCount = LimelightHelpers.getRawFiducials(m_cameraName).length;
    // if (Robot.count % 200 == 0) {
    //   logf("%s TV:%b tagCount:%d pX:%.3f pY:%.3f ", m_cameraName, tv, tagCount, pose.getX(), pose.getY());
    // }
    if (tv && tagCount >= 0 && (pose.getX() != 0 || pose.getY() != 0)) {
      cyclecount++;
      if (cyclecount >= 5) {
        m_swerveDrive.getSwerveDrive().addVisionMeasurement(pose, imageCaptureTime);
        cyclecount = 0;
      }
    } else {
      cyclecount = 0;
    }
  }

  /**
   * Get the current robot pose on the field.
   *
   * @throws NoValidPoseException when no valid pose is available
   * @return Optional<Pose2d> of the robot's position relative to the field
   */
  public Optional<Pose2d> getRobotPose() throws NoValidPoseException {
    try {
      return Optional.of(LimelightHelpers.getBotPose2d_wpiBlue(m_cameraName));
    } catch (Exception e) {
      throw new NoValidPoseException(e.getMessage());
    }
  }

  public Optional<Pose2d> getTargetPose() {
    try {
      if (LimelightHelpers.getTV(m_cameraName)) {
        int targetID = (int) LimelightHelpers.getFiducialID(m_cameraName);
        Pose2d tagPose = m_fieldLayout.getTagPose(targetID).orElseThrow().toPose2d();
        return Optional.of(tagPose);
      }
      return Optional.empty();
    } catch (Exception e) {
      logf("Cannot find valid target: %s", e.getMessage());
      return Optional.empty();
    }
  }

  public Optional<Pose2d> getCurrentTargetPose() {
    try {
      if (tv) {
        int targetID = (int) LimelightHelpers.getFiducialID(m_cameraName);
        Pose2d tagPose = m_fieldLayout.getTagPose(targetID).orElseThrow().toPose2d();
        return Optional.of(tagPose);
      }
      return Optional.empty();
    } catch (Exception e) {
      logf("Cannot find valid target: %s", e.getMessage());
      return Optional.empty();
    }
  }

  /**
   * Resets the robot's orientation in the Limelight's pose estimation to zero. This can be used to
   * correct for any drift in the Limelight's orientation estimation over time. Note that this does
   * not affect the robot's actual orientation or the SwerveDrive's odometry; it only resets the
   * reference
   */
  public void resetVisionOrientation() {
    LimelightHelpers.SetRobotOrientation(m_cameraName, 0, 0, 0, 0, 0, 0);
  }

  private void updateSmartDashboard() throws NoValidPoseException {
    // update robot pose
    Pose2d pose = getRobotPose().orElseThrow();
    publisher.set(pose);

      SmartDashboard.putNumber(m_shortName + ":PoseX", pose.getX());
      SmartDashboard.putNumber(m_shortName + ":PoseY", pose.getY());
      SmartDashboard.putNumber(m_shortName + ":PoseR", pose.getRotation().getDegrees());
      SmartDashboard.putNumber(
          m_shortName + ":Tags", LimelightHelpers.getTargetCount(m_cameraName));
    

    // update tag target info
    if (tv) {
      tagID = (int) LimelightHelpers.getFiducialID(m_cameraName);
     
      if (Robot.count % 250 == 100) {
        SmartDashboard.putNumber(m_shortName + ":ID", tagID);
        Pose2d tagPose = m_fieldLayout.getTagPose(tagID).orElseThrow().toPose2d();
        logf(
            "%s tag:%d Pose x:%.3f y:%.3f angle:%.3f delta x:%.3f y:%.3f angle:%.2f",
            m_shortName,
            tagID,
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees(),
            pose.getX() - tagPose.getX(),
            pose.getY() - tagPose.getY(),
            tagPose.getRotation().getDegrees());

        //   SmartDashboard.putNumber(m_shortName + ":TagPX", tagPose.getX());
        //   SmartDashboard.putNumber(m_shortName + ":TagPY", tagPose.getY());
        //   SmartDashboard.putNumber(m_shortName + ":TagPR", tagPose.getRotation().getDegrees() %
        // 360);
      }
    }
  }

  // Disabled 100-200
  // Enables
  public void setThrottle(int throtle) {
    LimelightHelpers.SetThrottle(m_cameraName, throtle);
  }
}

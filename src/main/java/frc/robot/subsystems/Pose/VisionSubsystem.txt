package frc.robot.subsystems.Pose;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightResults;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.LimelightHelpers.RawFiducial;

import java.util.Optional;

/** FRC robot code to interface with Limelight and display pose data. */
public class VisionSubsystem extends SubsystemBase {

  // --- Tuning constants ---
  private static final double MAX_TAG_AMBIGUITY = 0.7;
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;
  private static final double MIN_TAG_SPAN_METERS = 0.5; // only enforced with 2+ tags
  private static final double SINGLE_TAG_XY_STDDEV = 0.7;
  private static final double MULTI_TAG_XY_STDDEV = 0.3;
  private static final double ROT_STDDEV = 9999.0; // let gyro handle rotation
  private boolean testLog = false;

  // --- Identity ---
  private final String m_cameraName;
  private final String m_shortName;

  // --- Dependencies ---
  private final SwerveSubsystem m_swerveDrive;

  // --- Cached state (refreshed every periodic) ---
  private boolean m_mt_tv = false;
  private Pose2d m_pose = new Pose2d();
  private PoseEstimate m_mt = null;
  private RawFiducial[] m_rawFiducials;
  private double m_mt_yaw;
  int tagCount;

  // --- Field layout (shared across all instances) --- Actually this is loaded as
  // a separate instance for each camera, this is not global even though the info
  // contained should be
  private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);

  // --- NT publisher ---
  private final StructPublisher<Pose2d> m_publisher;

  public static class NoValidPoseException extends Exception {
    public NoValidPoseException(String msg) {
      super("Pose unavailable: " + msg);
    }
  }

  public VisionSubsystem(SwerveSubsystem swerveDrive, String cameraName, String shortName) {
    m_swerveDrive = swerveDrive;
    m_cameraName = cameraName;
    m_shortName = shortName;

    setName(m_cameraName);

    // LimelightHelpers.SetIMUMode(m_cameraName, 3); //2 of our 3 cameras don't have
    // IMUs so this is hurting more than helping.
    LimelightHelpers.SetIMUMode(m_cameraName, 0); // 0 means robot only uses pidgeon for imu measurments

    if (shortName == "rear")
      LimelightHelpers.SetFiducialIDFiltersOverride(m_cameraName,
          new int[] { 2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27 });

    m_publisher = NetworkTableInstance.getDefault()
        .getStructTopic(m_shortName + ":MyPose", Pose2d.struct)
        .publish();
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // tagCount = LimelightHelpers.getRawFiducials(m_cameraName).length;
    tagCount = LimelightHelpers.getTargetCount(m_cameraName);
    m_rawFiducials = LimelightHelpers.getRawFiducials(m_cameraName);
    try {
      // First, tell Limelight your robot's current orientation
      double robotYaw = m_swerveDrive.getPose().getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation(m_cameraName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

      // this set of variables was never implemented?
      // Pose2d robotPose = m_swerveDrive.getPose();
      // double yaw = pigeon.getYaw().getValueAsDouble();
      // double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

      // Get pose estimate from camera
      m_mt = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cameraName);
      m_mt_yaw = m_mt.pose.getRotation().getDegrees();
      m_mt_tv = LimelightHelpers.getTV(m_cameraName);

      SmartDashboard.putNumber(m_shortName + "-TagCount", tagCount);
      SmartDashboard.putBoolean(m_shortName + "-TV", m_mt_tv);

      // This should not be here it is trippling the resource requirements by logging
      // the same info 3 times every single cycle
      // if (m_mt != null) {
      // SmartDashboard.putNumber("Swerve Heading",
      // m_swerveDrive.getPose().getRotation().getDegrees());
      // SmartDashboard.putNumber("Pigeon Heading",
      // pigeon.getYaw().getValueAsDouble());
      // SmartDashboard.putNumber(m_shortName + " VHead", m_mt_yaw);
      // }

      if (m_mt_tv && m_mt != null && tagCount > 0) {
        // Cache pose once so all helpers share the same snapshot.
        m_pose = m_mt.pose;
        updateSwerveOdometry(m_mt);
        updateSmartDashboard();
      }

    } catch (Exception e) {
      logf("Error in VisionSubsystem periodic (%s): %s", m_cameraName, e.getMessage());
    }
  }

  // ---------------------------------------------------------------------------
  // Odometry update
  // ---------------------------------------------------------------------------

  /**
   * Applies the vision measurement to the swerve pose estimator.
   *
   * Quality gates (all must pass):
   * 1. At least one tag visible
   * 2. Pose not at origin (Limelight default when no fix)
   * 3. Tag not too far away
   * 4. Multi-tag span not too narrow (if 2+ tags)
   */
  private void updateSwerveOdometry(PoseEstimate mt) {
    Pose2d pose = mt.pose;

    if (tagCount == 1 && m_rawFiducials.length == 1) {
      if (m_rawFiducials[0].ambiguity > MAX_TAG_AMBIGUITY) {
        return;
      }
      if (m_rawFiducials[0].distToCamera > MAX_TAG_DISTANCE_METERS) {
        return;
      }
    }

    // Gate 1 – origin guard (if camera claims robot at 0,0 data is bad)
    if (pose.getX() == 0.0 && pose.getY() == 0.0)
      return;

    // Gate 2 – distance
    if (mt.avgTagDist > MAX_TAG_DISTANCE_METERS)
      return;

    // Gate 3 – multi-tag span (skip for single-tag; span is 0)
    if (tagCount >= 2 && mt.tagSpan < MIN_TAG_SPAN_METERS)
      return;

    double gyroRate = m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond;

    if (Math.abs(gyroRate) > 360) {
      return;
    }

    // Use the timestamp that comes directly from the PoseEstimate — it is
    // already the FPGA-adjusted capture time (pipeline + capture latency baked in).
    double captureTimestamp = mt.timestampSeconds;

    // Scale std devs by distance: further = less trust.
    double distScale = 1.0 + mt.avgTagDist * 0.3;
    double xyStdDev = (tagCount >= 2 ? MULTI_TAG_XY_STDDEV : SINGLE_TAG_XY_STDDEV)
        * distScale;
    Matrix<N3, N1> stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(),
        xyStdDev, xyStdDev, ROT_STDDEV);

    m_swerveDrive.getM_swerveDrive().addVisionMeasurement(pose, captureTimestamp, stdDevs);
  }

  // ---------------------------------------------------------------------------
  // Public accessors
  // ---------------------------------------------------------------------------

  /** Returns a human-readable summary of the latest vision result. */
  public String getVisionResult() {
    if (m_mt_tv) {
      return String.format(
          "%s Tag:%d X:%.3f Y:%.3f R:%.2f",
          m_shortName,
          (int) LimelightHelpers.getFiducialID(m_cameraName),
          m_pose.getMeasureX().in(Meters),
          m_pose.getMeasureY().in(Meters),
          m_pose.getRotation().getDegrees());
    }
    return m_shortName + " — no valid vision result";
  }

  /**
   * Returns the robot's estimated field pose from the most recent Limelight
   * frame.
   * Uses the cached PoseEstimate; never triggers a new NT read.
   */
  public Optional<Pose2d> getRobotPose() {
    if (m_mt == null || !m_mt_tv || tagCount == 0)
      return Optional.empty();
    Pose2d p = m_mt.pose;
    if (p.getX() == 0.0 && p.getY() == 0.0)
      return Optional.empty();
    return Optional.of(p);
  }

  /**
   * Returns the field pose of the currently targeted AprilTag, if any.
   * Uses cached tv so this is safe to call frequently.
   */
  // what is the point of this? especially since we know the position of all april
  // tags always, and cannot distinguish which april tage to return in the method
  public Optional<Pose2d> getCurrentTargetPose() {
    if (!m_mt_tv)
      return Optional.empty();
    try {
      int id = (int) LimelightHelpers.getFiducialID(m_cameraName);
      return FIELD_LAYOUT.getTagPose(id).map(p3d -> p3d.toPose2d());
    } catch (Exception e) {
      logf("Cannot resolve target pose (%s): %s", m_cameraName, e.getMessage());
      return Optional.empty();
    }
  }

  /**
   * Same as getCurrentTargetPose() but performs a fresh TV check.
   * Use this if you need the target pose outside of periodic context.
   */
  public Optional<Pose2d> getTargetPose() {
    if (!LimelightHelpers.getTV(m_cameraName))
      return Optional.empty();
    return getCurrentTargetPose();
  }

  // ---------------------------------------------------------------------------
  // Camera control
  // ---------------------------------------------------------------------------

  public void switchPipeline(int index) {
    LimelightHelpers.setPipelineIndex(m_cameraName, index);
  }

  public double getPipeline() {
    LimelightResults results = LimelightHelpers.getLatestResults(m_cameraName);
    return results.pipelineID;
  }

  /** 0 = full speed, up to 100 = minimal frame rate. */
  public void setThrottle(int throttle) {
    LimelightHelpers.SetThrottle(m_cameraName, throttle);
  }

  /**
   * Resets the Limelight's robot orientation reference to zero.
   * Call this after a known-heading reset (e.g. auto start).
   */
  public void resetVisionOrientation() {
    LimelightHelpers.SetRobotOrientation(m_cameraName, 0, 0, 0, 0, 0, 0);
  }

  // ---------------------------------------------------------------------------
  // SmartDashboard
  // ---------------------------------------------------------------------------

  private void updateSmartDashboard() {
    m_publisher.set(m_pose);

    if (testLog) {
      SmartDashboard.putNumber(m_shortName + ":PoseX", m_pose.getX());
      SmartDashboard.putNumber(m_shortName + ":PoseY", m_pose.getY());
      SmartDashboard.putNumber(m_shortName + ":PoseR", m_pose.getRotation().getDegrees());
      SmartDashboard.putNumber(m_shortName + ":AvgTagDist",
          m_mt != null ? m_mt.avgTagDist : -1);
    }

    if (m_mt_tv && Robot.count % 250 == 100) {
      int tagID = (int) LimelightHelpers.getFiducialID(m_cameraName);
      SmartDashboard.putNumber(m_shortName + ":ID", tagID);
      try {
        Pose2d tagPose = FIELD_LAYOUT.getTagPose(tagID).orElseThrow().toPose2d();
        logf(
            "%s tag:%d pose x:%.3f y:%.3f deg:%.1f  delta x:%.3f y:%.3f",
            m_shortName, tagID,
            m_pose.getX(), m_pose.getY(), m_pose.getRotation().getDegrees(),
            m_pose.getX() - tagPose.getX(),
            m_pose.getY() - tagPose.getY());
      } catch (Exception e) {
        logf("%s tag %d not found in field layout", m_shortName, tagID);
      }
    }
  }

  // this is not supposed to be inside the camera
  // public void resetGyroYaw() {
  // if (m_mt == null || !m_mt_tv || tagCount == 0) {
  // logf("Cannot reset gyro yaw from %s: no valid pose", m_shortName);
  // return;
  // }
  // m_swerveDrive.resetYaw(m_mt_yaw);
  // }

  public double getVisionYaw() {
    return m_mt_yaw;
  }
}

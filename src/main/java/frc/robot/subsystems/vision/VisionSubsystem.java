package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.Optional;

/** FRC robot code to interface with Limelight and display pose data. */
public class VisionSubsystem extends SubsystemBase {

  // --- Tuning constants ---
  private static final double MAX_TAG_DISTANCE_METERS = 4.0;
  private static final double MIN_TAG_SPAN_METERS     = 0.5; // only enforced with 2+ tags
  private static final double SINGLE_TAG_XY_STDDEV    = 0.9;
  private static final double MULTI_TAG_XY_STDDEV     = 0.3;
  private static final double ROT_STDDEV              = 9999.0; // let gyro handle rotation
  private boolean testLog = false;

  // --- Identity ---
  private final String m_cameraName;
  private final String m_shortName;

  // --- Dependencies ---
  private final SwerveSubsystem m_swerveDrive;

  // --- Cached state (refreshed every periodic) ---
  private boolean       m_tv   = false;
  private Pose2d        m_pose = new Pose2d();
  private PoseEstimate  m_mt2  = null;
  private double m_mt2_yaw;

  // Initialize with the CAN ID configured in Phoenix Tuner X
  Pigeon2 pigeon = new Pigeon2(14, "canivore"); 

  // --- Field layout (shared across all instances) ---
  private static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // --- NT publisher ---
  private final StructPublisher<Pose2d> m_publisher;

  public static class NoValidPoseException extends Exception {
    public NoValidPoseException(String msg) { super("Pose unavailable: " + msg); }
  }

  public VisionSubsystem(SwerveSubsystem swerveDrive, String cameraName, String shortName) {
    m_swerveDrive = swerveDrive;
    m_cameraName  = cameraName;
    m_shortName   = shortName;

    setName(cameraName);

    LimelightHelpers.SetIMUMode(m_cameraName, 0);

    m_publisher = NetworkTableInstance.getDefault()
        .getStructTopic(m_shortName + ":MyPose", Pose2d.struct)
        .publish();
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    try {
      //Pose2d robotPose = m_swerveDrive.getPose();
      double yaw = pigeon.getYaw().getValueAsDouble();
      double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

      LimelightHelpers.SetRobotOrientation(
          m_cameraName,
          yaw,
          yawRate,
          0.0,
          0.0,
          0.0,
          0.0
      );
      // Fetch once per loop — reuse m_mt1 everywhere below.
      // m_mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cameraName);
      
      m_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);

      m_mt2_yaw = m_mt2.pose.getRotation().getDegrees();
      m_tv  = LimelightHelpers.getTV(m_cameraName);

      SmartDashboard.putBoolean(m_shortName + "-TV", m_tv);
      SmartDashboard.putNumber(m_shortName + "-TagCount",
           m_mt2 != null ? m_mt2.tagCount : 0);

      if (m_mt2 != null) {
        SmartDashboard.putNumber("Swerve Heading", m_swerveDrive.getPose().getRotation().getDegrees());
        // SmartDashboard.putNumber("Pigeon Heading", pigeon.getYaw().getValueAsDouble());
        SmartDashboard.putNumber(m_shortName + " VHead", m_mt2_yaw);
      }

      if (m_tv && m_mt2 != null && m_mt2.tagCount > 0) {
        // Cache pose once so all helpers share the same snapshot.
        m_pose = m_mt2.pose;
        updateSwerveOdometry(m_mt2);
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
   *   1. At least one tag visible
   *   2. Pose not at origin (Limelight default when no fix)
   *   3. Tag not too far away
   *   4. Multi-tag span not too narrow (if 2+ tags)
   */
  private void updateSwerveOdometry(PoseEstimate mt2) {
    Pose2d pose = mt2.pose;

    // Gate 1 – origin guard
    if (pose.getX() == 0.0 && pose.getY() == 0.0) return;

    // Gate 2 – distance
    if (mt2.avgTagDist > MAX_TAG_DISTANCE_METERS) return;

    // Gate 3 – multi-tag span (skip for single-tag; span is 0)
    if (mt2.tagCount >= 2 && mt2.tagSpan < MIN_TAG_SPAN_METERS) return;

    // Use the timestamp that comes directly from the PoseEstimate — it is
    // already the FPGA-adjusted capture time (pipeline + capture latency baked in).
    double captureTimestamp = mt2.timestampSeconds;

    // Scale std devs by distance: further = less trust.
    double distScale = 1.0 + mt2.avgTagDist * 0.3;
    double xyStdDev = (mt2.tagCount >= 2 ? MULTI_TAG_XY_STDDEV : SINGLE_TAG_XY_STDDEV)
                      * distScale;
    Matrix<N3, N1> stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(),
        xyStdDev, xyStdDev, ROT_STDDEV);
    
    double gyroRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

    if (Math.abs(gyroRate) > 360) {
        return;
    }

    m_swerveDrive.getM_swerveDrive().addVisionMeasurement(pose, captureTimestamp, stdDevs);
  }

  // ---------------------------------------------------------------------------
  // Public accessors
  // ---------------------------------------------------------------------------

  /** Returns a human-readable summary of the latest vision result. */
  public String getVisionResult() {
    if (m_tv) {
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
   * Returns the robot's estimated field pose from the most recent Limelight frame.
   * Uses the cached PoseEstimate; never triggers a new NT read.
   */
  public Optional<Pose2d> getRobotPose() {
    if (m_mt2 == null || !m_tv || m_mt2.tagCount == 0) return Optional.empty();
    Pose2d p = m_mt2.pose;
    if (p.getX() == 0.0 && p.getY() == 0.0) return Optional.empty();
    return Optional.of(p);
  }

  /**
   * Returns the field pose of the currently targeted AprilTag, if any.
   * Uses cached tv so this is safe to call frequently.
   */
  public Optional<Pose2d> getCurrentTargetPose() {
    if (!m_tv) return Optional.empty();
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
    if (!LimelightHelpers.getTV(m_cameraName)) return Optional.empty();
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
          m_mt2 != null ? m_mt2.avgTagDist : -1);
    }

    if (m_tv && Robot.count % 250 == 100) {
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

  public void resetGyroYaw() {
    if (m_mt2 == null || !m_tv || m_mt2.tagCount == 0) {
      logf("Cannot reset gyro yaw from %s: no valid pose", m_shortName);
      return;
    }
    m_swerveDrive.resetYaw(m_mt2_yaw);
  }

  public double getVisionYaw() {
    return m_mt2_yaw;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.intake.IntakeTilt;
import frc.robot.subsystems.shooter.CatchupSubsystem;
import frc.robot.subsystems.shooter.DrumstickSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.PositionSubsystem;
import frc.robot.subsystems.hotDog.HotDog;
import frc.robot.subsystems.intake.IntakeSpin;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.FieldConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemV2;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_driverController = new CommandXboxController(0);
  final CommandXboxController m_opController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/Sibling_2026"));

  private VisionSubsystemV2 m_visionLeft;
  private VisionSubsystemV2 m_visionRight;
  private VisionSubsystemV2 m_visionRear;

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;
  private final IntakeTilt intakeTilt;
  private final IntakeSpin intakeSpin;
  private final CatchupSubsystem catchupSubsystem;
  private final DrumstickSubsystem drumstickSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final PositionSubsystem positionSubsystem; 
  private final HotDog hotDog; 

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getM_swerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.4)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_driverController::getRightX,
          m_driverController::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getM_swerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.4)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_driverController.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              m_driverController.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_visionLeft = new VisionSubsystemV2(m_drivebase, "limelight-left", "left");
    m_visionRight = new VisionSubsystemV2(m_drivebase, "limelight-right", "right");
    m_visionRear = new VisionSubsystemV2(m_drivebase, "limelight-rear", "rear");
    intakeSpin = new IntakeSpin();
    intakeTilt = new IntakeTilt();
    catchupSubsystem = new CatchupSubsystem();
    drumstickSubsystem = new DrumstickSubsystem();
    hoodSubsystem = new HoodSubsystem();
    positionSubsystem = new PositionSubsystem(m_drivebase);
    hotDog = new HotDog(); 

    // Configure the trigger bindings
    configureBindings();
    driverBindings();
    operatorBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    autoChooser.addOption("Drive Forward", m_drivebase.driveForward().withTimeout(1));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = m_drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = m_drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = m_drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = m_drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      m_driverController.start()
          .onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      m_driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    if (DriverStation.isTest()) {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      m_driverController.start().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      m_driverController.back().whileTrue(m_drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    }
  }

  private void driverBindings() {
    m_driverController.a().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
    m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().whileTrue(m_drivebase.centerModulesCommand());
    m_driverController.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());

    m_driverController
        .start()
        .onTrue(
            Commands.runOnce(() -> m_drivebase.resetOdometry(getInitPose()))
                .andThen(myLogf("Reset Pose to 1.2,1.2,180")));

    m_driverController.rightTrigger().whileTrue(
        m_drivebase.aimAtPoseCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.HUB_POSE_BLUE
                : FieldConstants.HUB_POSE_RED,  false, 0)
            .alongWith(myLogf("Aiming at hub pose:" +
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? "Blue Hub"
                    : "Red Hub"))));
    m_driverController.x().onTrue(Commands.runOnce(SignalLogger::start));
    m_driverController.b().onTrue(Commands.runOnce(SignalLogger::stop));

//commands you need to run sysid. run the logger, then quasistatic forward, reverse; dynamic forward, reverse; end log
//     m_joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
// m_joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

// /*
//  * Joystick Y = quasistatic forward
//  * Joystick A = quasistatic reverse
//  * Joystick B = dynamic forward
//  * Joystick X = dyanmic reverse
//  */
// m_joystick.y().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
// m_joystick.a().whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
// m_joystick.b().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
// m_joystick.x().whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // PositionAndShootCommand
    //m_driverController.leftTrigger().whileTrue(new PositionAndShootCommand(position, drumstick, hood, catchup, swerve));
  }

  private void operatorBindings(){
    m_opController.button(1).whileTrue(drumstickSubsystem.setShooterSetpoint(2000));
    m_opController.button(2).whileTrue(drumstickSubsystem.stopShooter());

    m_opController.button(3).whileTrue(catchupSubsystem.setCatchupSetpoint(600));
    m_opController.button(4).whileTrue(catchupSubsystem.stopCatchup());

    m_opController.button(5).whileTrue(hoodSubsystem.setHoodPosition(1));
    m_opController.button(6).whileTrue(hoodSubsystem.setHoodPosition(0));

    m_opController.button(7).whileTrue(new InstantCommand(() -> intakeTilt.zeroEncoder())); // To figure out rotations needed for extension
    m_opController.button(8).whileTrue(intakeTilt.setIntakeTiltSetpoint(120));
    m_opController.button(9).whileTrue(new InstantCommand(() -> intakeTilt.homeIntake()));

    m_opController.button(10).whileTrue(intakeSpin.intakeSpin(2000))
        .whileFalse(intakeSpin.stopIntake());
    m_opController.button(11).whileTrue(hotDog.setVelocitySetpoint(1000)).whileFalse(hotDog.stop());

    
    // m_opController.button(12).whileTrue(intakeSpin.intakeSpin(2000))
    //     .whileFalse(intakeSpin.stopIntake());
    // m_opController.button(13).whileTrue(drumstickSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_opController.button(14).whileTrue(drumstickSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_opController.button(15).whileTrue(drumstickSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_opController.button(16).whileTrue(drumstickSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
   m_opController.button(13).whileTrue(drumstickSubsystem.setShooterSetpoint(positionSubsystem.getShooterRPM()));
  }

  private Pose2d getInitPose() {
    if (Robot.isAllianceBlue()) {
      return new Pose2d(new Translation2d(1.2, 1.2), Rotation2d.fromDegrees(-180));
    } else {
      return new Pose2d(new Translation2d(15.341, 1.2), Rotation2d.fromDegrees(180));
    }
  }

  public void switchPipelines(int num) {
    m_visionLeft.switchPipeline(num);
    m_visionRight.switchPipeline(num);
    m_visionRear.switchPipeline(num);

  }

  public void setVisionThrottle(int throttle) {
    m_visionLeft.setThrottle(throttle);
    m_visionRight.setThrottle(throttle);
    m_visionRear.setThrottle(throttle);
  }

  private void logPoses() {
    logf("LL ******* Pose: %s %s %s Robot:%s", m_visionLeft.getVisionResult());
    logf("LRight******* Pose: %s %s %s Robot:%s", m_visionRight.getVisionResult());
    logf("LRear******* Pose: %s %s %s Robot:%s", m_visionRear.getVisionResult());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }

  public Command myLogf(String pattern, Object... args) {
    return new InstantCommand(() -> logf(pattern, args));
  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  public void homing() {
    intakeTilt.zeroEncoder();
  }
}
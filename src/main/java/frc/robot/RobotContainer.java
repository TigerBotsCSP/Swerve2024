// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AttachmentCoordinator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.attachment.FeederSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem;
import frc.robot.subsystems.attachment.ShooterSubsystem;
import frc.robot.subsystems.attachment.UTBIntakerSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem.PivotPosition;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Autogenerated chooser with all the auto routes
  public final SendableChooser<Command> autoChooser;

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final AttachmentCoordinator m_attatchment = new AttachmentCoordinator(
      new UTBIntakerSubsystem(),
      new FeederSubsystem(),
      new ShooterSubsystem(),
      new PivotSubsystem());

  // The driver's controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_attachmentController = new CommandXboxController(OIConstants.kAttatchmentsControllerPort);

  // Fields for visualization and testing
  private final Field2d m_field = new Field2d();
  private final Field2d m_estimationField = new Field2d();

  public RobotContainer() {
    registerPathplannerCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Pose Estimation", m_estimationField);
    SmartDashboard.putNumber("Pivot Angle", PivotConstants.kAmpPos);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));

  }

  /**
   * Register named commands used in pathplanner autos
   */
  private void registerPathplannerCommands() {
    // TODO: Auto Commands
    NamedCommands.registerCommand("spinShooter", m_attatchment.getSpinShooterCommand());
    NamedCommands.registerCommand("continuousFire", m_attatchment.getContinuousFireCommand());
    NamedCommands.registerCommand("subwooferPosition", m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));
    NamedCommands.registerCommand("intakePositon", m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Base controls

    // Set X wheels
    m_driverController.rightBumper().whileTrue(Commands.run(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Auto aiming
    m_driverController.leftTrigger().whileTrue(Commands.run(
        () -> m_robotDrive.driveWithHeading(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            getAimingVector(getTarget()).getAngle(),
            true, false),
        m_robotDrive));

    // Reset field oriented
    m_driverController.x().onTrue(Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }));

    // D-pad turning
    m_driverController.povUp().whileTrue(Commands.run(() -> m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        Rotation2d.fromDegrees(0), true, false),
        m_robotDrive));

    m_driverController.povRight().whileTrue(Commands.run(() -> m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        Rotation2d.fromDegrees(-90), true, false),
        m_robotDrive));

    m_driverController.povDown().whileTrue(Commands.run(() -> m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        Rotation2d.fromDegrees(180), true, false),
        m_robotDrive));

    m_driverController.povLeft().whileTrue(Commands.run(() -> m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        Rotation2d.fromDegrees(90), true, false),
        m_robotDrive));

    // Attatchment controls

    // Intake
    m_attachmentController.b().or(m_driverController.b()).whileTrue(m_attatchment.getIntakeCommand());

    // Unjam
    m_attachmentController.y().or(m_driverController.y()).whileTrue(m_attatchment.getUnjamIntakersCommand());

    // Spin up shooter
    m_attachmentController.leftBumper().whileTrue(m_attatchment.getSpinShooterCommand());

    // Shoot
    m_attachmentController.rightTrigger().or(m_driverController.rightTrigger())
        .onTrue(m_attatchment.getShootCommand());

    // Arm/pivot positioning

    m_attachmentController.povUp().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));

    m_attachmentController.povDown().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));

    m_attachmentController.povLeft().onTrue(m_attatchment.getSetCustomPivotPositionCommand(SmartDashboard.getNumber("Pivot Angle", 0)));

    m_attachmentController.povRight().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));
  }

  public Translation2d getTarget() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? TargetConstants.kBlueSpeakerTarget : TargetConstants.kRedSpeakerTarget;
  }

  public Translation2d getAimingVector(Translation2d target) {
    return m_robotDrive.getPose().getTranslation().minus(target);
  }

  public void periodic() {
    SmartDashboard.putString("Attatchment State", switch (m_attatchment.getState()) {
      case kAiming -> "Aiming";
      case kShooting -> "Shooting";
      case kContinuousFire -> "Auto Continuous";
    });

    SmartDashboard.putBoolean("Beam Break", m_attatchment.getBeamBreakState());

    m_field.setRobotPose(m_robotDrive.getPose());

    var result = VisionConstants.rearCam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    if (target != null) {
      var pose = VisionConstants.rearCamPoseEstimator.update();

      if (pose.isPresent()) {
        Pose2d estimatedPose = pose.get().estimatedPose.toPose2d();
        double timestamp = pose.get().timestampSeconds;

        m_estimationField.setRobotPose(estimatedPose);
        m_robotDrive.updateOdometryWithVision(estimatedPose, timestamp);

        double distance = getAimingVector(getTarget()).getNorm();
        SmartDashboard.putNumber("auto angle distance", distance);
      } else {
        m_estimationField.setRobotPose(new Pose2d());
      }
    }

    double angle = m_robotDrive.getHeading().getDegrees()
        - getAimingVector(getTarget()).getAngle().getDegrees();
    SmartDashboard.putNumber("auto angle diff", angle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

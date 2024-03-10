// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AttachmentCoordinator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.attachment.FeederSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem;
import frc.robot.subsystems.attachment.PivotSubsystem.PivotPosition;
import frc.robot.subsystems.attachment.ShooterSubsystem;
import frc.robot.subsystems.attachment.UTBIntakerSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Autogenerated chooser with all the auto routes
  public final SendableChooser<Command> autoChooser;

  // Other (tests)
  double m_targetDistance = 0;
  boolean m_autoAim = false;

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(() -> {
    if (m_autoAim) {
      return Optional.of(getTargetVector().getAngle());
    } else {
      return Optional.empty();
    }
  });

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

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftY()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
  }

  /**
   * Register named commands used in pathplanner autos
   */
  private void registerPathplannerCommands() {
    NamedCommands.registerCommand("startContinuousFire", m_attatchment.getStartContinuousFireCommand());
    NamedCommands.registerCommand("stopContinuousFire", m_attatchment.getStopContinuousFireCommand());

    // TODO: maybe find a cleaner way to implement this. also includes running the
    // pivot function constantly during auto
    NamedCommands.registerCommand("startAutoAim", Commands.runOnce(() -> {
      m_autoAim = true;
    }).asProxy());

    NamedCommands.registerCommand("stopAutoAim", Commands.runOnce(() -> {
      m_autoAim = false;
    }).asProxy());

    NamedCommands.registerCommand("zeroGyro", Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }).asProxy());

    NamedCommands.registerCommand("pivotSubwoofer",
        m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));
    NamedCommands.registerCommand("pivotIntake",
        m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));

    NamedCommands.registerCommand("startFeeders", m_attatchment.getStartShootCommand());
    NamedCommands.registerCommand("startShooter", m_attatchment.getSpinShooterAutoCommand());
    NamedCommands.registerCommand("startIntakers", m_attatchment.getIntakeAutoCommand().asProxy());
    // will not stop
    NamedCommands.registerCommand("intake", m_attatchment.getIntakeCommand());

    NamedCommands.registerCommand("stopFeeders", m_attatchment.getStopShootCommand());
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // Rumble on intake note
    m_attatchment.bindControllerRumble(m_driverController);
    m_attatchment.bindControllerRumble(m_attachmentController);

    // Base controls

    // Set X wheels
    m_driverController.rightBumper().whileTrue(Commands.run(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Auto aiming
    m_attachmentController.rightTrigger().whileTrue(Commands.run(() -> {
      autoAimDrive(getAimingVector(getTarget()).getAngle());
      autoAimPivot();
    }));

    // Reset field oriented
    m_driverController.x().onTrue(Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }));

    // D-pad turning
    m_driverController.povUp().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(0, 180))), m_robotDrive));

    m_driverController.povRight().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(-90, 90))), m_robotDrive));

    m_driverController.povDown().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(180, 0))), m_robotDrive));

    m_driverController.povLeft().whileTrue(Commands.run(() -> autoAimDrive(Rotation2d.fromDegrees(getFromAlliance(90, -90))), m_robotDrive));

    // Attatchment controls

    // Intake
    m_attachmentController.b().or(m_driverController.b()).whileTrue(m_attatchment.getIntakeCommand());

    // Unjam
    m_attachmentController.y().or(m_driverController.y()).whileTrue(m_attatchment.getUnjamIntakersCommand());

    // Spin up shooter
    m_attachmentController.leftBumper().whileTrue(m_attatchment.getSpinShooterCommand());

    // Shoot
    m_driverController.rightTrigger()
        .whileTrue(m_attatchment.getStartShootCommand())
        .whileFalse(m_attatchment.getStopShootCommand());

    // Arm/pivot positioning

    m_attachmentController.povUp().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));

    m_attachmentController.povDown().onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kIntakePosition));

    m_attachmentController.povRight()
        .onTrue(m_attatchment.getSetPivotPositionCommand(PivotPosition.kSubwooferPosition));
  }

  public void autoAimPivot() {
    //double angle = (42.9919 * Math.pow(.601, m_targetDistance));
    // Auto aiming up-down
    //double angle = (35.5428 * Math.pow(.7066, m_targetDistance));
    double angle = (35.8266 * Math.pow(.7037, m_targetDistance));
/*
    // Angle adjustments
    double adjustment = distance;
    
    if (distance > 4.6) {
      adjustment *= .5;
    } else if (distance > 4.4) {
      adjustment *= .7;
    } else if (distance > 4.3) {
      adjustment *= .7;
    } else if (distance > 4.2) {
      adjustment *= .7;
    } else if (distance > 4.0) {
      adjustment *= .8;
    }

    angle -= adjustment;*/

    if (angle < 30 && angle > 0) {
      m_attatchment.setCustomPosition(angle);
    }
  }

  public void autoAimDrive(Rotation2d angle) {
    // Auto aiming left-right (offset is 5 degrees for alignment)
    m_robotDrive.driveWithHeading(
        -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftY()), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(invertIfRed(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
        angle,
        true, false);
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public <T> T getFromAlliance(T blueVal, T redVal) {
    return isBlueAlliance() ? blueVal : redVal;
  }

  public double invertIfRed(double num) {
     return num * getFromAlliance(1, -1);
  }

  public Translation2d getTarget() {
    return TargetConstants.AimingTarget.kSpeaker.getTarget(isBlueAlliance());
  }

  public Translation2d getAimingVector(Translation2d target) {
    return m_robotDrive.getPose().getTranslation().minus(target);
  }

  public Translation2d getTargetVector() {
    if (m_autoAim) {
      autoAimPivot();
    }
    return getAimingVector(getTarget());
  }

  public void periodic() {
    SmartDashboard.putNumber("auto aim distance", m_targetDistance);
    SmartDashboard.putBoolean("has note", m_attatchment.getBeamBreakState());
    SmartDashboard.putBoolean("auto aim", m_autoAim);

    m_field.setRobotPose(m_robotDrive.getPose());

    var pose = VisionConstants.rearCamPoseEstimator.update();

    if (pose.isPresent()) {
      Pose2d estimatedPose = pose.get().estimatedPose.toPose2d();
      double timestamp = pose.get().timestampSeconds;

      m_estimationField.setRobotPose(estimatedPose);
      m_robotDrive.updateOdometryWithVision(estimatedPose, timestamp);

      m_targetDistance = getAimingVector(getTarget()).getNorm();
    } else {
      m_estimationField.setRobotPose(new Pose2d());
    }
  }

  public void prepareTeleop() {
      m_attatchment.stopContinuousFire();
      m_autoAim = false;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Stop continuous fire and auto aim after auto ends
    return autoChooser.getSelected().finallyDo(() -> {
      // shooter stops too early for the third note sometimes
      //m_attatchment.stopContinuousFire();
     // m_autoAim = false;
    });
  }
}

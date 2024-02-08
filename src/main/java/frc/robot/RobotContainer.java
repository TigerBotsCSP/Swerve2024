// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.attachment.AttachmentHandler;
import frc.robot.subsystems.attachment.FeederSubsystem;
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

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final AttachmentHandler m_attatchment = new AttachmentHandler(
      new UTBIntakerSubsystem(),
      new FeederSubsystem(),
      new ShooterSubsystem());

  // The driver's controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_attachmentController = new CommandXboxController(OIConstants.kAttatchmentsControllerPort);

  public RobotContainer() {
    // Register auto commands
    NamedCommands.registerCommand("Wait 1s & Shoot", new WaitCommand(1));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
                true, true, false),
            m_robotDrive));

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

    m_driverController.rightBumper().whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    // Drive speeds
    m_driverController.rightTrigger().whileTrue(new RunCommand(
        () -> {
          Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.8 * 0.5;
        }))
        .whileFalse(new RunCommand(() -> {
          Constants.DriveConstants.kMaxSpeedMetersPerSecond = 4.8 * 1.3;
        }));

    // Reset field oriented
    m_driverController.x().onTrue(new InstantCommand(() -> {
      m_robotDrive.resetGyro();
    }));

    // D-pad turning (trash)
    // Could try a run command that stops when angle is reached
    m_driverController.povUp().onTrue(new InstantCommand(() -> {
      ExecutorService executor = Executors.newFixedThreadPool(1);
      executor.submit(() -> {
        // Normalize the degree
        double angle = m_robotDrive.getHeading().getDegrees() % 360.0;

        while (true) {
          if (angle < 180) {
            m_robotDrive.drive(0, 0, -.45, false, false, false);
          } else {
            m_robotDrive.drive(0, 0, .45, false, false, false);
          }

          // 180 degrees
          if (Math.abs(angle) > 175 && Math.abs(angle) < 185) {
            m_robotDrive.drive(0, 0, 0, false, false, false);
            break;
          }

          angle = m_robotDrive.getHeading().getDegrees() % 360.0;
        }
      });
    }));

    // better turning
    m_driverController.povDown().whileTrue(new RunCommand(() -> {
      System.out.println(
          "abs pos: " + m_robotDrive.m_frontLeft.m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
      System.out.println("rel pos: " + m_robotDrive.m_frontLeft.m_turningEncoder.getPosition());
    }));

    // Attatchment controls

    m_attachmentController.rightBumper()
        .onTrue(m_attatchment.getSpinShooterCommand())
        .onFalse(m_attatchment.getStopShooterCommand());

    m_attachmentController.rightTrigger().onTrue(m_attatchment.getShootCommand());

    m_attachmentController.x().onTrue(m_attatchment.getStartIntakersCommand());
    m_attachmentController.a().onTrue(m_attatchment.getStopIntakersCommand());
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

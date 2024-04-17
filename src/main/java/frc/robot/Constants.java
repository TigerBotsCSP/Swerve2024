// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class UTBIntakerConstants {
    // Constants for the under the bumper intaker
    public static final int kIntakeMotor1CANId = 13;
    public static final int kIntakeMotor2CANId = 14;

    public static final double kIntakeMotorSpeed = 0.8;
    public static final double kReverseMotorSpeed = -0.8;
  }

  public static final class FeederConstants {
    // Constants for the feeder that sits between the intake and shooter
    public static final int kFeederMotorCANId = 10;

    public static final int kBeamBreakDIOId = 1;

    public static final double kIntakeSpeed = 0.9;
    public static final double kReverseSpeed = -0.9;
    public static final double kReverseAlignNoteSpeed = -0.3;
    public static final double kShootSpeed = 1;

    // max note pullabck time in seconds
    public static final double kNotePullbackMaxTime = 2;
  }

  public static final class PivotConstants {
    public static final int kLeftPivotMotorCANId = 9;
    public static final int kRightPivotMotorCANId = 15;

    public static final double kPivotGearRatio = 1.0 / 250; // 250 to 1 from motor to pivot
    public static final double kPivotSpeed = 0.5;

    // Pivot range is 0-60
    public static final double kGlobalMin = 2;
    public static final double kGlobalMax = 30;

    // Shooting is from the entire range
    public static final double kSpeakerMin = kGlobalMin;
    public static final double kSpeakerMax = kGlobalMax;

    // Speaker positions
    public static final double kSubwooferPos = 21; // left (good)
    public static final double kPodiumPos = 11; // up (good)
    public static final double kIntakePos = 7; // down (good) 11
    public static final double kAmpPos = 17; // 17

    // Distance before pivot is considered ready
    public static final double kPositionDeadzone = 2.0;

    public static final class PivotPID {
      public static final double kP = 0.16;
      public static final double kI = 1e-4;
      public static final double kD = 0.0058;
      public static final double kIz = 0;
      public static final double kFF = 0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
    }
  }

  public static final class ShooterConstants {
    // Constants for shooter
    public static final int kRightShooterMotorCANId = 11;
    public static final int kLeftShooterMotorCANId = 12;
    public static final int kAmpShooterMotorCANID = 16;

    // Shooting speed 0-1
    public static final double kShootSpeed = 1;
    public static final double kAmpShotSpeed = 0.4;

    // Shooting speed for subwoofer side
    public static final double kSubwooferSideShootSpeed = 1;

    // Shooter bottom multiplier
    public static final double kBottomSpeed = .8;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond = 4.8 * 1.3;// speeds
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kAutoAimMaxAngularSpeed = Units.radiansToDegrees(540.00);
    public static final double kAutoAimMaxAngularAccel = Units.radiansToDegrees(720.00);

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration (24x24)
    public static final double kTrackWidth = Units.inchesToMeters(22);
    // Distance between centers of right and left wheels on robot (24*24)
    public static final double kWheelBase = Units.inchesToMeters(18.5); // 18.5?
    // Distance from center to furthest wheel (*diagonal*)
    public static final double kCenterToWheel = Units.inchesToMeters(Math.sqrt(121 + 121)); // 11^2 + 11^2 PT
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kDiagonalMeters = Units.inchesToMeters(33.941);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = true;
    public static final double kGyroAdjustment = 0.0;

    public static final PIDConstants kTranslationPID = new PIDConstants(5, 0); // Translation PID constants
    public static final PIDConstants kRotationPID = new PIDConstants(5, 0, 0); // Rotation PID constants
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.365 / 1); // 3.365
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedMps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedMps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAttatchmentsControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    // PathFlowerConfig for PathPlanner's AutoBuilder
    public static final HolonomicPathFollowerConfig AutoPathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), // Translation PID constants
        new PIDConstants(5, 0, 0), // Rotation PID constants
        10.0, // Max module speed, in m/s
        DriveConstants.kCenterToWheel, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    // The layout of the apriltags for pose estimation
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final PhotonCamera rearCam = new PhotonCamera("BW3");

    // Note: Negative shifts up and left (relative to field, not up on the field2d)
    // x=-8.55 y=-6.5 (OUR FIELD VALUES, revert back after comps)
    // WORLDS: 5.55, -7.55 for red x
    public static final Transform3d rearCamOffset = new Transform3d(
        new Translation3d(Units.inchesToMeters(-5.55), Units.inchesToMeters(-5.5), -Units.inchesToMeters(-11)),
        new Rotation3d(0, Units.degreesToRadians(-35.5), Math.PI));
    public static final PhotonPoseEstimator rearCamPoseEstimator = new PhotonPoseEstimator(aprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCam, rearCamOffset);
            public static final PhotonPoseEstimator calibrationPoseEstimator = new PhotonPoseEstimator(aprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCam, rearCamOffset);
  }

  public static final class TargetConstants {
    // Apriltag #7
    // Calibrate at comps
    public static final Translation2d kBlueSpeakerTarget = new Translation2d(-0.04, 5.55);
    // Apriltag #4
    // Calibrate at comps
    public static final Translation2d kRedSpeakerTarget = new Translation2d(16.58, 5.55);

    // Apriltag #6
    // Calibrate at comps
    public static final Translation2d kBlueAmpTarget = new Translation2d(1.84, 8.20);
    // Apriltag #5
    // Calibrate at comps
    public static final Translation2d kRedAmpTarget = new Translation2d(14.70, 8.20);

    public enum AimingTarget {
      kSpeaker(kBlueSpeakerTarget, kRedSpeakerTarget),
      kAmp(kBlueAmpTarget, kRedAmpTarget);

      private Translation2d blue, red;

      AimingTarget(Translation2d blue, Translation2d red) {
        this.blue = blue;
        this.red = red;
      };

      public Translation2d getTarget(boolean isBlueAlliance) {
        return isBlueAlliance ? blue : red;
      }
    }
  }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO find these
    public static final double kMaxTurnSpeedRadPerSecond =
        2 * Math.PI; // 180 degrees per second // TODO

    // Distance between front and back wheels on robot (meters)
    public static final double kTrackWidth = .52; // 0.29 * 2
    // Distance between left and right wheels on robot (meters)
    public static final double kWheelBase = 0.58; // 0.29 * 2

    // Swerve module positions relative to robot center (meters)
    public static final Translation2d kFrontLeftLocation =
        new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kFrontRightLocation =
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    public static final Translation2d kBackLeftLocation =
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d kBackRightLocation =
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            DriveConstants.kFrontLeftLocation,
            DriveConstants.kFrontRightLocation,
            DriveConstants.kBackLeftLocation,
            DriveConstants.kBackRightLocation);
  }

  public static final class ModuleConstants {
    public static final int kCurrentLimit = 60;

    public static final double kWheelDiameterMeters = 0.1016; // 4 inches
    public static final double kDrivingMotorReduction =
        5.9; // 5.9:1 raido; // 6.86:1 // TODO find correct value
    public static final double kDriveWheelFreeSpeedRps =
        6784.0
            / 60; // 60.0; // NEO free speed in rotations per second TODO do we need to change this?

    public static final double kTurningP = 1.4;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;

    public static final double kDrivingP = 0.15;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 1.0;

    // Front Left Module CAN IDs
    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kFrontLeftTurningCanId = 11;

    // Front Right Module CAN IDs
    public static final int kFrontRightDrivingCanId = 22;
    public static final int kFrontRightTurningCanId = 12;

    // Back Left Module CAN IDs
    public static final int kBackLeftDrivingCanId = 23;
    public static final int kBackLeftTurningCanId = 13;

    // Back Right Module CAN IDs
    public static final int kBackRightDrivingCanId = 24;
    public static final int kBackRightTurningCanId = 14;
  }

  public static final class ShooterConstants {
    // Shooter motor CAN IDs
    public static final int kLeftShooterCanId = 31;
    public static final int kRightShooterCanId = 32;

    // Default shooter speed
    public static final double kDefaultShooterSpeed = 0.55;

    public static final double kP = .00025;
    public static final double kI = 0.00025;
    public static final double kD = 0.000125;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    public static final double kPXController = 0.4;
    public static final double kPYController = 0.4;
    public static final double kPThetaController = 0.4;

    public static final edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
        kThetaControllerConstraints =
            new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
  }

  public static final class VisionConstants {
    // Auto-aim PID constants
    public static final double kAutoAimKP = 9.0;
    public static final double kAutoAimKI = 0.9;
    public static final double kAutoAimKD = 0;
    public static final double kAutoAimMaxVelocity = 8.0; // rad/s
    public static final double kAutoAimMaxAcceleration = 20.0; // rad/s^2
    public static final double kAutoAimTolerance = 0.05; // radians (~3 degrees)

    // Goal/Target coordinates (in meters, WPILib blue origin)
    // Reefscape field is 16.54m x 8.21m
    // Blue alliance goal/processor is at the blue side (X=0)
    // Red alliance goal/processor is at the red side (X=16.54m)
    public static final double kBlueGoalX = 4.27;
    public static final double kBlueGoalY = 4.105; // Center of field Y
    public static final double kRedGoalX = 12.27;
    public static final double kRedGoalY = 4.105; // Center of field Y
  }

  public static final class IntakeConstants {
    public static final int kIntakeCANId = 36;
    public static final double kDefaultIntakeSpeed = 1;
    public static final int kCurrent = 30;
  }

  public static final class FeedConstants {
    public static final double kDefaultFeedSpeed = .7;
    public static final double kDefaultFingerSpeed = 1;
    public static final int kFeedCanId = 35;
    public static final int kSorterLeftCANId = 33;
    public static final int kSorterRightCANId = 34;
    public static final int kCurrent = 30;
  }

  public static final class ControlConstants {

    // Joystick USB ports
    public static final int kDriverLeftPort = 1;
    public static final int kDriverRightPort = 2;
    public static final int kButtonBoardPort = 3;

    public static final class FlightStickButtons {
      public static final int redThumbButton = 3;
      public static final int grayThumbButton = 20;
      public static final int grayPinkyButton = 5;
      public static final int grayTopButton = 4;
      public static final int redTriggerStageOne = 1;
      public static final int redTriggerStageTwo = 2;

      // these three buttons do not work
      public static final int bottomLeftButton = 0;
      public static final int bottomMiddleButton = 0;
      public static final int bottomRightButton = 0;

      public static final int topRightStickUp = 11;
      public static final int topRightStickDown = 13;
      public static final int topRightStickLeft = 14;
      public static final int topRightStickRight = 12;
      public static final int topRightStickMiddle = 15;

      public static final int middleStickUp = 6;
      public static final int middleStickDown = 8;
      public static final int middleStickLeft = 9;
      public static final int middleStickRight = 7;
      public static final int middleStickMiddle = 10;

      // all of these are in the POV section
      public static final int topLeftStickUp = 0;
      public static final int topLeftStickDown = 0;
      public static final int topLeftStickLeft = 0;
      public static final int topLeftStickRight = 0;
      public static final int topLeftStickMiddle = 0;
    }

    public static final class ButtonBoardButtons {}

    // Joystick axis deadband
    public static final double kJoystickDeadband = 0.05;

    // Slew rate limiter - units per second (1/3 sec from 0 to 1)
    public static final double kSlewRateLimit = 3.0;

    /*
        button board
        Toggle Switch is 11
        9 7 5 3 1
        10 8 6 4 2
        The joystick buttons
        1 = trigger first stage
        2 = trigger second stage
        3 = red thumb
        20 = left gray thumb
    */
    public static final int kIntakeExtendButton = FlightStickButtons.middleStickLeft;
    public static final int kIntakeRetractButton = FlightStickButtons.middleStickMiddle;
    public static final int kClimbExtendButton = FlightStickButtons.grayPinkyButton;
    public static final int kClimbRetractButton = FlightStickButtons.middleStickUp;
    public static final int kIntakeOnButton = FlightStickButtons.redThumbButton;
    public static final int kIntakeOffButton = FlightStickButtons.redTriggerStageOne;
    public static final int kIntakeReverseButton = FlightStickButtons.grayTopButton;
    public static final int kMaxOverdriveButton = FlightStickButtons.redTriggerStageTwo;
    public static final int kShooterSpeedUpButton = FlightStickButtons.middleStickRight;
    public static final int kShooterSpeedDownButton = FlightStickButtons.middleStickDown;
    public static final int kManualAimMode = FlightStickButtons.grayThumbButton;

    // left controller buttons
    public static final int kFieldToggleButton = FlightStickButtons.redTriggerStageOne;
    public static final int kResetFieldButton = FlightStickButtons.redThumbButton;
    public static final int kAutoAimButton = FlightStickButtons.grayThumbButton;
    public static final int kLeftPinkyButton = FlightStickButtons.grayPinkyButton;
    public static final int kUnclogButton = FlightStickButtons.grayTopButton;
    public static final int kRobotCentricButton = FlightStickButtons.grayTopButton;

    // left controller joystick
    public static final int kMoveXJoystick = 0; // side to side = axis 0
    public static final int kMoveYJoystick = 1; // front to back = axis 1

    // right controller buttons
    public static final int kRevShootButton = FlightStickButtons.grayThumbButton;
    public static final int kFeedButton = FlightStickButtons.redTriggerStageOne;
    public static final int kRightPinkyButton = FlightStickButtons.grayPinkyButton;

    // right controller joystick
    public static final int kRotateJoystick = 0;
  }

  public static final class ColorConstants {
    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color BLUE = new Color(0, 0, 255);
    public static final Color OFF = new Color(0, 0, 0);
  }
}

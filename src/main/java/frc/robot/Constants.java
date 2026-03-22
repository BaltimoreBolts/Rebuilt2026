// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ControllerConstants {
    public static final double kDeadzone = .1;
    public static final double kTriggerDeadzone = .05;

    public static final class Axis {
      public static final int kLeftX = 0;
      public static final int kLeftY = 1;
      public static final int kRightX = 4;
      public static final int kLeftTrigger = 2;
      public static final int kRightTrigger = 3;
      public static final int kRightY = 5;
    }

    public static final class Button {
      public static final int kA = 1;
      public static final int kB = 2;
      public static final int kX = 3;
      public static final int kY = 4;
      public static final int kLeftBumper = 5;
      public static final int kRightBumper = 6;
      public static final int kLeftMenu = 7;
      public static final int kRightMenu = 8;
      public static final int kLeftTriggerButton = 9;
      public static final int kRightTriggerButton = 10;
    }

    public static final class DPad {
      public static final int kUp = 0;
      public static final int kRight = 90;
      public static final int kDown = 180;
      public static final int kLeft = 270;
    }
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // TODO: get current robot mass
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

  /* The following classes are subjected to change based on CAN IDs */

  public static class SwerveDriveConstants {
    public static final int kFrontLeftDrivePort = 2;
    public static final int kFrontLeftRotPort = 3;
    public static final int kFrontRightDrivePort = 4;
    public static final int kFrontRightRotPort = 5;

    public static final int kBackLeftDrivePort = 8;
    public static final int kBackLeftRotPort = 9;
    public static final int kBackRightDrivePort = 6;
    public static final int kBackRightRotPort = 7;

    /* Given future growth for the encoders */
    public static final int kFrontLeftEncoder = -999;
    public static final int kFrontRightEncoder = -999;

    public static final int kBackLeftEncoder = -999;
    public static final int kBackRightEncoder = -999;

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class IntakeConstants {
    public static final int kIntakeRollerPort = 10;
    public static final int kIntakeDeployPort = 11;
  }

  public static class IndexerConstants {
    public static final int kIndexerPort = 12;

    /* Given future growth */
    public static final int kClimberDIO = -999;
  }

  public static class ShooterConstants {
    // positve -> intake
    public static final int kShooterKickerPort = 13;
    public static final int kShooterLeftFlyWheelPort = 14;
    public static final int kShooterRightFlyWheelPort = 15;
    public static final int kShooterHoodPort = 16;
  }
}

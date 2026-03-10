// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final double MAX_SPEED = Units.feetToMeters(14.5);


  /* The following classes are subjected to change based on CAN IDs */

  public static class SwerveDriveConstants {
    public static final int kFrontLeftDrivePort = 2;
    public static final int kFrontLeftRotPort = 3;
    public static final int kFrontRightDrivePort = 4;
    public static final int kFrontRightRotPort = 5;

    public static final int kBackLeftDrivePort = 6;
    public static final int kBackLeftRotPort = 7;
    public static final int kBackRightDrivePort = 8;
    public static final int kBackRightRotPort = 9;

    /* Given future growth */
    public static final int kFrontLeftEncoder = -999;
    public static final int kFrontRightEncoder = -999;

    public static final int kBackLeftEncoder = -999;
    public static final int kBackRightEncoder = -999;
  }

  public static class IntakeConstants {
    public static final int kIntakeRollerPort = 10;
    public static final int kIntakeDeployPort = 11;
  }

  public static class ClimberConstants {
    public static final int kClimberPort = 12;

    /* Given future growth */
    public static final int kClimberDIO = -999;
  }

  public static class ShooterConstants {
    // positve -> intake
    public static final int kShooterIntakePort = 13;
    public static final int kShooterLeftFlyWheelPort = 14;
    public static final int kShooterRightFlyWheelPort = 15;
    public static final int kShooterHoodPort = 16;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  private final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);
  private final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4);

  public SwerveSubsystem() {
    // Create modules (FL, FR, BL, BR order)
    var fl = createModule(
        new SparkMax(SwerveDriveConstants.kFrontLeftDrivePort, MotorType.kBrushless),
        new SparkMax(SwerveDriveConstants.kFrontLeftRotPort, MotorType.kBrushless),
        new CANcoder(3),
        "frontleft",
        new Translation2d(Inches.of(12), Inches.of(12)));
    
    var fr = createModule(
        new SparkMax(SwerveDriveConstants.kFrontRightDrivePort, MotorType.kBrushless),
        new SparkMax(SwerveDriveConstants.kFrontRightRotPort, MotorType.kBrushless),
        new CANcoder(6),
        "frontright",
        new Translation2d(Inches.of(12), Inches.of(-12)));
    
    var bl = createModule(
        new SparkMax(SwerveDriveConstants.kBackLeftDrivePort, MotorType.kBrushless),
        new SparkMax(SwerveDriveConstants.kBackRightDrivePort, MotorType.kBrushless),
        new CANcoder(9),
        "backleft",
        new Translation2d(Inches.of(-12), Inches.of(12)));
    
    var br = createModule(
        new SparkMax(SwerveDriveConstants.kBackRightDrivePort, MotorType.kBrushless),
        new SparkMax(SwerveDriveConstants.kBackRightRotPort, MotorType.kBrushless),
        new CANcoder(12),
        "backright",
        new Translation2d(Inches.of(-12), Inches.of(-12)));

    // Create SwerveDriveConfig
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        // .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
  }

  private AngularVelocity maximumAngularVelocity = DegreesPerSecond.of(360);
private LinearVelocity maximumLinearVelocity = MetersPerSecond.of(4);

public SwerveInputStream getChassisSpeedsSupplier(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier rotation) 
{
  return new SwerveInputStream(drive, translationX, translationY, rotation)
      .withMaximumAngularVelocity(maximumAngularVelocity)
      .withMaximumLinearVelocity(maximumLinearVelocity)
      .withDeadband(0.01)
      .withCubeRotationControllerAxis()     // Non-linear rotation response
      .withCubeTranslationControllerAxis()  // Non-linear translation response
      .withAllianceRelativeControl();       // Alliance-aware field-relative
}

  public SwerveModule createModule(
    SparkMax driveMotor, 
    SparkMax azimuthMotor, 
    CANcoder absoluteEncoder, 
    String moduleName,
    Translation2d location) 
{
  // Define gearing ratios
  MechanismGearing driveGearing = new MechanismGearing(12.75);  // L2 SDS MK4i
  MechanismGearing azimuthGearing = new MechanismGearing(6.75);
  Distance wheelDiameter = Inches.of(4);

  // Drive motor configuration
  SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
      .withWheelDiameter(wheelDiameter)
      .withClosedLoopController(0.3, 0, 0)
      .withGearing(driveGearing)
      .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / 4.5, 0.01))  // kS, kV, kA
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

  // Azimuth motor configuration
  SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 1))
      .withGearing(azimuthGearing)
      .withStatorCurrentLimit(Amps.of(20))
      .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

  // Create SmartMotorControllers
  SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
  SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);

  // Create module configuration
  SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
      .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
      .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
      .withLocation(location)
      .withOptimization(true);  // Enable state optimization

  return new SwerveModule(moduleConfig);
}


/**
 * Drive with robot-relative chassis speeds.
 */
public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier) {
  return drive.drive(speedsSupplier);
}

/**
 * Drive to a specific pose.
 */
public Command driveToPose(Pose2d pose) {
  return drive.driveToPose(pose);
}

public SwerveInputStream getDriveInput(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot)
        .withMaximumAngularVelocity(maxAngularVelocity)
        .withMaximumLinearVelocity(maxLinearVelocity)
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl();
  }

  public Command drive(Supplier<ChassisSpeeds> speeds) {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speeds.get()));
  }

  public Command lock() { return run(drive::lockPose); }
  public Pose2d getPose() { return drive.getPose(); }
  public void resetOdometry(Pose2d pose) { drive.resetOdometry(pose); }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }
}
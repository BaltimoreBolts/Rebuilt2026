// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.0;

  private SmartMotorControllerConfig intakePivotSmartMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          // Feedforward Constantsrad
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // Change to actual pivot Gear Ratio
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(10))
          .withClosedLoopRampRate(Seconds.of(0.1))
          .withOpenLoopRampRate(Seconds.of(0.1));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(11, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController intakePivotController =
      new SparkWrapper(spark, DCMotor.getNEO(1), intakePivotSmartMotorConfig);

  private ArmConfig intakePivotConfig =
      new ArmConfig(intakePivotController)
          // Soft limit is applied to the SmartMotorControllers PID
          .withSoftLimits(Degrees.of(0), Degrees.of(80))
          // Hard limit is applied to the simulation.
          .withHardLimit(Degrees.of(-10), Degrees.of(90))
          // Starting position is where your arm starts
          .withStartingPosition(Degrees.of(80))
          // TODO: Add Real Length and Mass
          .withLength(Feet.of(1))
          .withMass(Pounds.of(2))
          // Telemetry name and verbosity for the arm.
          .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm intakePivot = new Arm(intakePivotConfig);

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   *
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return arm.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
   * setpoint.
   *
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle) {
    return arm.runTo(angle, Degrees.of(2));
  }

  /**
   * Move the arm up and down.
   *
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  /** Run sysId on the {@link Arm} */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}

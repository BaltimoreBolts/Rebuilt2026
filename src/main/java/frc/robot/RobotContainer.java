// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final KickerSubsystem m_kickerSubsystem = new KickerSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController m_driverController = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> m_driverController.getLeftY() * -1,
              () -> m_driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY(),
              () -> -m_driverController.getLeftX())
          .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Default command forces arm to go to 0 degrees
    m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.setAngle(Degrees.of(0)));
    // Set the default command to force the shooter rest.
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.set(0));
    m_kickerSubsystem.setDefaultCommand(m_kickerSubsystem.set(0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_shooterSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_shooterSubsystem));

    m_driverController.start().onTrue(m_intakeSubsystem.rezero());

    // Bumpers drop and intake or outake
    m_driverController.leftBumper().whileTrue(m_intakeSubsystem.backFeedAndRollCommand());
    m_driverController.rightBumper().whileTrue(m_intakeSubsystem.deployAndRollCommand());

    // D-pad up and down manually control the intake roller speed
    m_driverController.povUp().whileTrue(m_intakeSubsystem.set(0.35));
    m_driverController.povDown().whileTrue(m_intakeSubsystem.set(-0.35));
    // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController
        .leftTrigger()
        .whileTrue(
            m_shooterSubsystem.shootCommand(RPM.of(60), m_kickerSubsystem, m_indexerSubsystem));
    m_driverController
        .rightTrigger()
        .whileTrue(
            m_shooterSubsystem.shootCommand(RPM.of(300), m_kickerSubsystem, m_indexerSubsystem));
    // Schedule 'set' when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.x().whileTrue(m_shooterSubsystem.set(0.3));
    // m_driverController.y().whileTrue(m_shooterSubsystem.set(-0.3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.runOnce(drivebase::zeroGyro);
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}

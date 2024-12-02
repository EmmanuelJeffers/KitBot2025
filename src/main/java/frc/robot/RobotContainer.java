// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driver = new CommandPS5Controller(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driver.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driver.getLeftX(),
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driver.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driver.getHID()::getCrossButtonPressed,
                                                                 driver.getHID()::getTriangleButtonPressed,
                                                                 driver.getHID()::getCircleButtonPressed,
                                                                 driver.getHID()::getSquareButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driver.getRightX(),
      () -> driver.getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driver.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driver.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driver.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driver.getRawAxis(2) * -1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    addCommandsToDashboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    if (DriverStation.isTest())
    {
      driver.circle().whileTrue(drivebase.sysIdDriveMotorCommand());
      driver.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driver.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driver.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driver.create().whileTrue(drivebase.centerModulesCommand());
      driver.L1().onTrue(Commands.none());
      driver.R1().onTrue(Commands.none());
      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    } else
    {
      driver.cross().onTrue(Commands.none());
      driver.square().onTrue(Commands.none());
      driver.circle().whileTrue(Commands.none());
      driver.triangle().whileTrue(Commands.none());
      driver.L2().whileTrue(drivebase.aimAtSpeaker(2));
      driver.options().whileTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driver.create().whileTrue((Commands.runOnce(drivebase::zeroGyro)));
      driver.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driver.R1().onTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
      drivebase.setDefaultCommand(
          !RobotBase.isSimulation() ? closedAbsoluteDriveAdv/*driveFieldOrientedAnglularVelocity*/ : closedAbsoluteDriveAdv);
    }
  }

  public void addCommandsToDashboard() {
    SmartDashboard.putData("Aim at Speaker", drivebase.aimAtSpeaker(2));
    SmartDashboard.putData("Zero Gyro", Commands.runOnce(drivebase::zeroGyro));
    SmartDashboard.putData("Lock Wheels", Commands.runOnce(drivebase::lock));
    SmartDashboard.putData("Drive Home", Commands.deferredProxy(() -> drivebase.driveToPose(
      new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    SmartDashboard.putData("", closedAbsoluteDriveAdv);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new PathPlannerAuto("TestAuto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

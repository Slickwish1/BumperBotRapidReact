// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.tools.Diagnostic;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AimToBall;
import frc.robot.commands.AimToBallManualDist;
import frc.robot.commands.AimToTarget;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSystem robotDrive = new DriveSystem();
  private JoystickButton left_bumper;
  private JoystickButton right_bumper;
  private JoystickButton a_button;

  private GenericHID driveController =  new XboxController(0) ;

  private SendableChooser teamColor = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    robotDrive.resetGyro();

    teamColor.addOption("Red", false);
    teamColor.addOption("Blue", true);

    robotDrive.setDefaultCommand( 

      (new RunCommand(
        () -> 
             robotDrive.drive(
              -(Math.abs(driveController.getRawAxis(1)) >0.1? driveController.getRawAxis(1):0.0),
              -(Math.abs(driveController.getRawAxis(0)) >0.1? driveController.getRawAxis(0):0.0)),
               robotDrive))
               
               //.alongWith(new SimpleIntakeOnVar(intakeSystem, (driveController.getRawAxis(3)))
    
    
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    left_bumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    right_bumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    a_button = new JoystickButton(driveController, XboxController.Button.kA.value);


    left_bumper.whileHeld(new AimToTarget(robotDrive));
    right_bumper.whileHeld(new AimToBall(robotDrive, (Boolean) teamColor.getSelected()));
    a_button.whenPressed(new AutoCommand(robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

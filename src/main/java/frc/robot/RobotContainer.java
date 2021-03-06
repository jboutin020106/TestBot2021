// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinCommand;
import frc.robot.subsystems.MotorSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final MotorSubsystem motorSub;

  private final SpinCommand spinCom;

  private final JoystickButton driveButton;

  private Joystick driveStick;

  private final ElevatorSubsystem elevatorSub;
  
  private JoystickButton elevatorButton;

  private final ElevatorCommand elevatorMove;

  private JoystickButton hDriveButton;

  private JoystickButton toggleButton;
  
  private Command toggleHDrive;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driveStick = new Joystick(Constants.driveStick);

    toggleButton = new JoystickButton(driveStick, Constants.toggleButton);

    driveButton = new JoystickButton(driveStick, Constants.driveButton);

    elevatorButton = new JoystickButton(driveStick, Constants.elevatorButton);

    hDriveButton = new JoystickButton(driveStick, Constants.hDriveButton);
  
    motorSub = new MotorSubsystem();

    elevatorSub = new ElevatorSubsystem();

    
    elevatorMove = new ElevatorCommand(elevatorSub);

    elevatorButton = new JoystickButton(driveStick, Constants.elevatorButton);

    hDriveButton = new JoystickButton(driveStick, Constants.hDriveButton);
  
  
    spinCom = new SpinCommand(motorSub, driveStick);
    motorSub.setDefaultCommand(spinCom);

  
    toggleHDrive = new InstantCommand(motorSub::changeToggle,motorSub);
    
   
   
   
  



    configureButtonBindings();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
  elevatorButton.whileHeld(elevatorMove);
  toggleButton.whenPressed(toggleHDrive);
  


    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}

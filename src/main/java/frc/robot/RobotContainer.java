// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final MotorSubsystem motorSub;

  private final SpinCommand spinCom;

  private final JoystickButton driveButton;

  private Joystick driveStick;

  private final ElevatorSubsystem elevatorSub;
  
  private JoystickButton elevatorButton;

  private final ElevatorCommand elevatorMove;
  
  private final JoystickButton hDriveButton;

  private JoystickButton toggleButton;

  private JoystickButton rotateToAngleButton;
  
  private Command toggleHDrive;

  private Command runAuto;

  private Command rotateToNinety;

  private Command driveRobot;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driveStick = new Joystick(Constants.driveStick);

    toggleButton = new JoystickButton(driveStick, Constants.toggleButton);

    driveButton = new JoystickButton(driveStick, Constants.driveButton);

    elevatorButton = new JoystickButton(driveStick, Constants.elevatorButton);

    hDriveButton = new JoystickButton(driveStick, Constants.hDriveButton);

    rotateToAngleButton = new JoystickButton(driveStick, Constants.rotateToAngleButton);
  
    motorSub = new MotorSubsystem();

    elevatorSub = new ElevatorSubsystem();
    
    elevatorMove = new ElevatorCommand(elevatorSub);

    elevatorButton = new JoystickButton(driveStick, Constants.elevatorButton);


  
    spinCom = new SpinCommand(motorSub, driveStick);
    motorSub.setDefaultCommand(spinCom);

  
    toggleHDrive = new InstantCommand(motorSub::changeToggle,motorSub);

    runAuto = new AutonomousCommand(motorSub);

    rotateToNinety = new RotateToAngleCommand(90, motorSub);

    driveRobot = new SpinCommand(motorSub, driveStick);


    
   
   
   
  



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
    rotateToAngleButton.whenPressed(rotateToNinety);
    driveButton.whileHeld(driveRobot);
    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return runAuto;
  }
}

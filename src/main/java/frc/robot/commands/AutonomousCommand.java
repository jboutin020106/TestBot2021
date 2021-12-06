// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MotorSubsystem;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
    private final MotorSubsystem motorSubAuto;


  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(MotorSubsystem motorSubTemp) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    motorSubAuto = motorSubTemp;
    addCommands(
      new AutonomousSpin(motorSubAuto, 0.5, 0.5).withTimeout(0.8), 
      new AutonomousSpin(motorSubAuto, 0.8, 0.8).withTimeout(0.8),
      new AutonomousSpin(motorSubAuto, 1, 1).withTimeout(0.8)
      

      
    );
    
    
  }
}

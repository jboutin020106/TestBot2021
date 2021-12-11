// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

public class MotorSubsystem extends SubsystemBase {
  private WPI_TalonSRX leftSide;
  private WPI_TalonSRX rightSide;
  private WPI_TalonSRX hDriveMotor;
  private AHRS navX;
  private boolean toggle = false;

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    rightSide = new WPI_TalonSRX(Constants.rightMotor);
    leftSide = new WPI_TalonSRX(Constants.leftMotor);
    hDriveMotor = new WPI_TalonSRX(Constants.HDriveMotor);
    navX = new AHRS();
    rightSide.setInverted(true);
  
  

  }
    
  
    public AHRS getNavX(){
      return navX;
    }

    public void zeroNavX(){
      navX.reset();
    }

    public double getNavXAngle(){
      System.out.println(navX.getYaw());
      return navX.getYaw();
    }

  




 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinMotor(double rightSpeed, double leftSpeed){
    if(!toggle){
    leftSide.set(ControlMode.PercentOutput, leftSpeed);
    rightSide.set(ControlMode.PercentOutput, rightSpeed);
    }
    else if(toggle){
    hDriveMotor.set(ControlMode.PercentOutput, rightSpeed);
    }
  }
  public void changeToggle(){
    toggle = !toggle;
    System.out.println(toggle);
  }


}

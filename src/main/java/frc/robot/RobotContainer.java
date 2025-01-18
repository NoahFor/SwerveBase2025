// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;



public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //Driver CONTROLLER ON PORT 0 AND Utilities ON PORT 1
  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);


  public RobotContainer(){
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
      () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
      configureButtonBindings();
  }
  private void configureButtonBindings() {

  }
  }

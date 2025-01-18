// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauilabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMarx turningMotor;
  // built in encoders
  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;
  //to move angle motor
  private PIDController turningPidController;
  //absolute encoder so the wheel position can be kept constantly
  //is only used at the very beginning to reset relative encoders
  private CANcoder absoluteEncoder;
  
  private boolean absoluteEncoderReversed;
  //offset position
  //used to componsate for encoder error
  private double absoluteEncoderOffsetRad;
  private int turningMotorId;
  private int absoluteEncoderId;
  
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

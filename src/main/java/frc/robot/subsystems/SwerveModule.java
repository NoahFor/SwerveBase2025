// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax turningMotor;
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

private SparkMaxConfig config;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      config = new SparkMaxConfig();
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      this.absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
      turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
      this.turningMotorId = turningMotorId;
      this.absoluteEncoderId=absoluteEncoderId;

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = turningMotor.getEncoder();

      config
        .inverted(driveMotorReversed)
        .idleMode(IdleMode.kCoast);
      config.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

      config
        .inverted(turningMotorReversed)
        .idleMode(IdleMode.kCoast);
      config.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ModuleConstants.kPTurning, 0, 0);
      turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      }
  //gets drive encoder position in meters
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }
  //gets turning encoder position in radians
  public double getTurningPosition(){
    return turningEncoder.getPosition();
  }
  //gets swerve module position
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
  }
  //gets drive encoder velocity in m/s
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  //gets turning encoder velocity in rad/s
  public double getTurningVelocity(){
    return turningEncoder.getVelocity();
  }
//
public double getAbsoluteEncoderRad(){
  //getAbsolutePosition returns value from -0.5 to 0.5 (circle, -0.5 is next to 0.5)
  double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  //boom angle is now in radians
  angle *=(2*Math.PI);
  //offsets the angle according to typed constants
  //tip: use Pheonix tuner x to zero the absolute encoders manually instead
  //Every time absolute encoders are unplugged they need to be re-zeroed
  angle -= absoluteEncoderOffsetRad;
  //if they are reversed turn it negative

  if(absoluteEncoderReversed){
    return angle*(-1.0);
  }
  else
    return angle;
  }

public double getAbsoluteEncoderReading(){
  return getAbsoluteEncoderRad();
}

public void resetEncoders(){
  //zeros drive encoder
  driveEncoder.setPosition(0);
  turningEncoder.setPosition(getAbsoluteEncoderRad());
}
//gets absolute position of absolute encoders on range of -0.5 to 0.5 (raw data, no conversions)
public double getAbsolutePos(){
  return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
}
//gets the state of the module
public SwerveModuleState getState(){
  //creates a current state from the current drive velocity and turning position
  return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
}

public void setDesiredState(SwerveModuleState state){
  //does not set a state if there is no speed in the new state
  if(Math.abs(state.speedMetersPerSecond)<0.001){
    stop();
    return;
  }
  //optimize makes it so that the wheel only has to turn a max of 90 degrees at a time
  //uses both directions of drive motor instead of just forward
  state = SwerveModuleState.optimize(state, getState().angle);
  //sets the driveMotor speed and scale down using physical max meters per second of the motor
  driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  //sets the turningMotor position using a pid controller with inputs of the current position of turning motor
  //combined with the desired position
  turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  //
  //SmartDashboard.putString("Swerve[" + turningMotorId+"] state", state.toString());

}
//stop
public void stop(){
  driveMotor.set(0);
  turningMotor.set(0);
}
}

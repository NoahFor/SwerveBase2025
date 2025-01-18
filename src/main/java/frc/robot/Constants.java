package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants{
    //MotorConversionFactors
    // public static final double kDriveEncoderRot2Meter = 0;
    // public static final double kDriveEncoderRPM2MeterPerSec = 0;
    // public static final double kTurnEncoderRot2Rad = 0;
    // public static final double kTurnEncoderRPM2RadPerSec = 0;
  }
  public static class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1/6.75;
    public static final double kTurningMotorGearRatio = (1/(150.0/7));
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

    //Used as position conversion factor
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio *2* Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60;

    public static final double kPTurning = 0.3;
  }
  public static class ArmConstants{
    
    public static final double kArmMotorGearRatio = (1/5.0)*(1/4.0)*(1/3.0)*(16/66.0)*1.0;
    public static final double kArmAbsoluteGearRatio = (16/66.0)*1.0;
    public static final double kArmEncoderRot2Rad = kArmMotorGearRatio*2*Math.PI;
    public static final double kArmAbsoluteRot2Rad = kArmAbsoluteGearRatio*2*Math.PI;
    public static final double kArmEncoderRPM2RadPerSec = kArmEncoderRot2Rad/60;
    public static final double kArmAbsoluteRPM2RadPerSec = kArmAbsoluteRot2Rad/60;
    //check
    public static final int kArmMotorPort = 42;
    public static final boolean kArmMotorReversed = true;
    public static final double kRadLimitBot = -0.1;
    public static final double kRadLimitTop = 1.801266;
    //Angles for the four presets 
    public static final double kAmpAngle = 1.801266;
    public static final double kHerdAngle = 0;
    // public static final double kSpeakerCloseAngle = 0.476566;
    public static final double kSpeakerCloseAngle = 0.622599105834961;

    public static final double kSpeakerFarAngle = 0.682725;

    //
    //7ft from bumper to goal

    //test
    public static final double kForwardSpeed = 0.1;
    public static final double kBackwardSpeed = -0.1;
    //test
    public static final double kPArm = 0.7;
    public static final double kIArm = 0.04;
    public static final double kDArm = 0.01;
  }

  public static class FlyConstants{
    public static final double kFlyWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kFlyEncoderRot2Meter = Math.PI * kFlyWheelDiameterMeters;
    public static final double kFlyEncoderRPM2MeterPerSec = kFlyEncoderRot2Meter/60;
    //check
    public static final int kRightFlyMotorPort = 30;
    public static final int kLeftFlyMotorPort = 24;
    public static final boolean kRightFlyMotorReversed = false;
    public static final boolean kLeftFlyMotorReversed = true;

    public static final double kFlyRightMotorSpeed = 1;
    public static final double kFlyLeftMotorSpeed = 1;
    
    
  }
  public static class HerderConstants{
    public static final double kHerderMotorGearRatio = (1/4)*(1/3)*1.0;
    public static final double kHerderWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kHerderEncoderRot2Meter = kHerderMotorGearRatio*Math.PI * kHerderWheelDiameterMeters;
    public static final double kHerderEncoderRPM2MeterPerSec = kHerderEncoderRot2Meter/60;
    //check
    public static final int kTopHerderMotorPort = 51;
    public static final int kBotHerderMotorPort = 50;
    public static final boolean kTopHerderMotorReversed = false;
    public static final boolean kBotHerderMotorReversed = true;

    public static final double kHerderMotorSpeed = 0.75;
    
  }

  public static class WinchConstants{
    
    //check
    public static final double kWinchMotorGearRatio = 1.0;
    public static final double kWinchWheelDiameterMeters = Units.inchesToMeters(1);
    public static final double kWinchEncoderRot2Meter = kWinchMotorGearRatio*Math.PI * kWinchWheelDiameterMeters;
    public static final double kWinchEncoderRPM2MeterPerSec = kWinchEncoderRot2Meter/60;

    public static final double kMeterLimitBot = 0;
    public static final double kMeterLimitTop = 16;
    // public static final double kMeterLimitBot = -100;
    // public static final double kMeterLimitTop = 100;
    public static final int kWinchMotorPort = 40;
    public static final boolean kWinchMotorReversed = false;
    public static final double kWinchForwardSpeed =0.7;
    public static final double kWinchBackwardSpeed = 0.7;

    public static final double kPWinch = 0.1;
  }
  public static class DriveConstants{
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
      );



    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.47;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    
    
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 8;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 23;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
    //ZERO CANCODERS USING PHOENIX TUNER X INSTEAD
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; //21
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; //20
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; //23
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;  //22
  }
  public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;
        //yea idk how to tune these
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

  public static final class OIConstants {
    public static final int kDriverControllerOnePort = 0;
    public static final int kDriverControllerTwoPort = 1;

    //CHECK CONTROLLER VALUES
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxisXbox = 4;
    public static final int kDriverRotAxisJoystick = 2;


    ///////////////////////buttons\\\\\\\\\\\\\\\\\
    //////////////////////////Controller One\\\\\\\\\\\\\\\\\\\\\
    public static final int kDriverFieldOrientedButtonIdx =1;
    //x
    public static final int kDriverResetArmButton = 2;
    //B
    public static final int kRestGyrobutton = 3;
    //Y
    public static final int kShootSequenceButton = 4;
    //right Trigger
    public static final int kExtendLiftButton = 6;
   //left trigger
    public static final int kRetractLiftButton = 5;
    /////////////////////////////////Controller Two\\\\\\\\\\\\\\
    //Y
    public static final int kFlyWheelFwdButton = 4;
    //A
    public static final int kFlyWheelBwdButton = 1;
    //right trigger
    public static final int kArmForwardButton = 5;
    //left trigger
    public static final int kArmBackwardButton = 6;
    //B
    public static final int kHerderInButton = 3;
    //X
    public static final int kHerderOutButton = 2;

    //Plus up (POV button in degrees)
    public static final int kArmHerdButton = 0;
    //Plus Down
    public static final int kArmCloseSpeakerButton = 180;
    //plus right
    public static final int kArmAmpButton = 90;
    //plus left
    public static final int kArmFarSpeakerButton = 270;
    
    
    


    

    public static final double kDeadband = 0.1;
}
  public static final class LimeConstants{
    //in inches
    public static final double kSpeakerTagHieght = 53.88+3.25;
    public static final double kCameraHieght = 18.5;
    public static final double kCameraAngle = 10.0;
    public static final double kPTwistController = 2.4;
    public static final double kDTwistController = 0.009;
    public static final double kPDriveController = 0.1;
    public static final double kDDriveController = 0;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
          DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5.0;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    public static final TrapezoidProfile.Constraints kTwistControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

    public static final TrapezoidProfile.Constraints kDriveControllerConstraints = 
                new TrapezoidProfile.Constraints(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
  }
  
}
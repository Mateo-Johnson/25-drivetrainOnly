package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are the just allowed maximum speeds
    public static final double maxSpeedMetersPerSecond = 4.8; // Adjust based on preference
    public static final double maxAngularSpeed = 2 * Math.PI; // Radians per second 

    // Chassis configuration
    public static final double trackWidth = Units.inchesToMeters(29);
    public static final double wheelBase = Units.inchesToMeters(29);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // These values should be adjusted based on the Thrifty encoder readings
    public static final double frontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double frontRightChassisAngularOffset = 0;
    public static final double backLeftChassisAngularOffset = Math.PI;
    public static final double backRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs AND THRIFTY DIO PORTS FOR SWERVE 
    //FRONT LEFT
    public static final int frontLeftDrivingCanId = 10;
    public static final int frontLeftTurningCanId = 9;
    public static final int frontLeftEncoderPort = 1;

    //REAR LEFT
    public static final int rearLeftDrivingCanId = 8;
    public static final int rearLeftTurningCanId = 7;
    public static final int rearLeftEncoderPort = 2;

    //FRONT RIGHT
    public static final int frontRightDrivingCanId = 4;
    public static final int frontRightTurningCanId = 3;
    public static final int frontRightEncoderPort = 0;

    //REAR RIGHT
    public static final int rearRightDrivingCanId = 5; 
    public static final int rearRightTurningCanId = 6;
    public static final int rearRightEncoderPort = 3;


    public static final boolean gyroReversed = false;
  }

  public static final class ModuleConstants {
    // MK4i Configuration - L3 Gear Ratio
    public static final double wheelDiameterMeters = 0.10033; // MK4i wheel diameter
    public static final double drivingMotorReduction = 6.12; // L3 ratio
    
    // Calculations required for driving motor conversion factors and feed forward
    public static final double drivingMotorFreeSpeedRps = NeoMotorConstants.freeSpeedRpm / 60;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
        / drivingMotorReduction;

    // Turn motor encoder resolution
    public static final double turningMotorReduction = 150.0/7.0; // MK4i turning reduction
    
    // Thrifty encoder constants
    public static final double encoderResolution = 1.0; // Full rotation
    public static final double minEncoderFrequency = 100.0; // Hz - for detecting disconnection
    public static final double maxEncoderFrequency = 1000.0; // Hz - typical working range
  }

  public static final class OIConstants {
    public static final int primaryPort = 0;
    public static final int secondaryPort = 1; // Do we even need a secondary? Who knows
    public static final double driveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 4.75;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double freeSpeedRpm = 5676;
  }

}
package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    public static final int frontLeftDrivingCanId = 11;
    public static final int frontLeftTurningCanId = 10;
    public static final int frontLeftEncoderPort = 0;

    //REAR RIGHT
    public static final int rearLeftDrivingCanId = 13;
    public static final int rearLeftTurningCanId = 12;
    public static final int rearLeftEncoderPort = 2;

    //FRONT RIGHT
    public static final int frontRightDrivingCanId = 15;
    public static final int frontRightTurningCanId = 14;
    public static final int frontRightEncoderPort = 1;

    //REAR RIGHT
    public static final int rearRightDrivingCanId = 17;
    public static final int rearRightTurningCanId = 16;
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

  // public static class ElevatorConstants {
    // public static final int leftElevatorID = 1;
    // public static final int rightElevatorID = 2;
    // public static final int limitSwitchPort = 0;

    // // PID Values
    // public static final double kP = 0.00001;
    // public static final double kI = 0.0;
    // public static final double kD = 0.0;
    // public static final double kIz = 0.0;
    // public static final double kFF = 0.0;
    // public static final double kS = 0.0; // Static friction
    // public static final double kG = 0.0; // Gravity feed forward
    // public static final double kV = 0.0; // Velocity feed forward
    // public static final double kA = 0.0; // Acceleration feed forward

    // // Mechanism constants
    // public static final double gearRatio = 100.0;
    // public static final double encoderCountsPerRev = 42.0;
    // public static final double sprocketTeech = 22.0;
    // public static final double chainPitch = 0.25; // Given by the number in the chain (we use #25)
    
    // // Calculated constants
    // public static final double inchesPerSprocketRotation = sprocketTeech * chainPitch;
    // public static final double countsPerSprocketRotation = encoderCountsPerRev * gearRatio;
    // public static final double countsPerInch = countsPerSprocketRotation / inchesPerSprocketRotation;

    // // Position setpoints in inches
    // public static final double downPos = 0.0;
    // public static final double L1 = 12.0;
    // public static final double Intake = 14;
    // public static final double L2 = 24.0;
    // public static final double L3 = 36.0;
    // public static final double L4 = 48.0;
    // public static final double posTolerance = 0.5;
    // public static final double bottomPos = 0.1; // CHANGE THIS
    

    // // Safety limits
    // public static final double max_output = 0.5;
    // public static final double minPos = downPos;
    // public static final double maxPos = 50.0;
    // public static final double maxVelocity = 30.0;
    // public static final double maxAcceleration = 20.0; 
  // }

  public static class AlgaeConstants {
    public static final int topAlgaeID = 18;
    public static final int bottomAlgaeID = 19;
    public static final int algaePivotID = 20;
  }

  public static class CoralConstants {
    // Control
    public static double targetVelocity = 0.0;
    public static IntakeMode currentMode = IntakeMode.STOPPED;

    // Constants
    public static final int leftCoralID = 2; // Update this to match CAN ID
    public static final double gearRatio = 5.0; // Update based on gearing

    // Speed Constants (in RPM at the roller)
    public static final double intake_speed = 1000.0;
    public static final double eject_speed = -1000.0;
    public static final double hold_speed = 100.0;
    
    // PID Constants for velocity control
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.000185;

    // Current limiting
    public static final int SCL = 25; // amps
    public static final int FCL = 35; // amps
    
    public enum IntakeMode {
        INTAKING,
        EJECTING,
        HOLDING,
        STOPPED
    }
  }

  public static class LightConstants {
    public static int port = 1;
    public static int length = 30;
  }
  
  public final class ElevatorConstants {
    // Hardware
    public static int switchPort = 0;

    // Control
    public static  ProfiledPIDController profiledPIDController;
    public static  PIDController maintainPositionPID;
    public static  boolean isManualControl = false;
    public static  double currentSetpoint = 0.0;

    // Constants
    public static final int leadID = 1;
    public static final int followID = 1;
    public static final double gearRatio = 100.0;
    public static final double SPD = 1.391; // 22T #25 sprocket in inches
    
    // Setpoints in inches
    public static final double L1 = 0.0;
    public static final double intake = 24.0;
    public static final double L2 = 48.0;
    public static final double L3 = 64.0;
    public static final double L4 = 72.0;

    
    // Motion Profile Constraints
    public static final double maxV = 60.0; // inches per second
    public static final double maxA = 80.0; // inches per second squared
    
    // PID Constants
    public static final double prof_kP = 0.1;
    public static final double prof_kI = 0.0;
    public static final double prof_kD = 0.0;
    
    public static final double stay_kP = 0.05;
    public static final double stay_kI = 0.0;
    public static final double stay_kD = 0.0;
    
    // Manual Control
    public static final double max = 76.0; // max height | inches - slightly above highest setpoint
}
}
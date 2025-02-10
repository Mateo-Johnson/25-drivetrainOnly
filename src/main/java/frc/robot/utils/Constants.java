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

    // SPARK MAX CAN IDs AND THRIFTY DIO PORTS FOR SWERVE 
    //FRONT LEFT
    public static final int frontLeftDrivingCanId = 10;
    public static final int frontLeftTurningCanId = 9;
    public static final int frontLeftEncoderPort = 1;
    public static final double frontLeftChassisAngularOffset = 0;

    //REAR LEFT
    public static final int rearLeftDrivingCanId = 8;
    public static final int rearLeftTurningCanId = 7;
    public static final int rearLeftEncoderPort = 2;
    public static final double rearLeftChassisAngularOffset = 0;

    //FRONT RIGHT
    public static final int frontRightDrivingCanId = 4;
    public static final int frontRightTurningCanId = 3;
    public static final int frontRightEncoderPort = 0;
    public static final double frontRightChassisAngularOffset = 4;

    //REAR RIGHT
    public static final int rearRightDrivingCanId = 5; 
    public static final int rearRightTurningCanId = 6;
    public static final int rearRightEncoderPort = 3;
    public static final double rearRightChassisAngularOffset = 0;

        public static final boolean gyroReversed = false;
    }

    /**
     * Operator Interface constants
     */
    public static final class InterfaceConstants {
        public static final int primaryPort = 0;
        public static final double driveDeadband = 0.05;
        public static final double slowFactor = 0.5; // Speed factor to reduce drive speed (e.g., 50% speed)
    }

    /**
     * Autonomous mode constants
     */
    public static final class AutoConstants {
        // Motion Constraints
        public static final double maxSpeedMetersPerSecond = 4.75;
        public static final double maxAccelerationMetersPerSecondSquared = 3;
        public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

        // PID Controllers
        public static final double PXController = 1;
        public static final double PYController = 1;
        public static final double PThetaController = 1;

        // Trajectory Constraints
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                maxAngularSpeedRadiansPerSecond, 
                maxAngularSpeedRadiansPerSecondSquared
            );
    }
} //😔
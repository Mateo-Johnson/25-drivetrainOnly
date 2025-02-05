package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    /**
     * Motor-specific constants that apply across the robot
     */
    public static final class NeoMotorConstants {
        public static final double freeSpeedRpm = 5676;
    }

    /**
     * Swerve drive module-specific constants
     */
    public static final class ModuleConstants {
        // MK4i Physical Configuration (L3 Gear Ratio)
        public static final double wheelDiameterMeters = 0.10033;
        public static final double drivingMotorReduction = 6.12;
        public static final double turningMotorReduction = 150.0/7.0;
        
        // Thrifty Encoder Configuration
        public static final double encoderResolution = 1.0;
        public static final double minEncoderFrequency = 100.0;
        public static final double maxEncoderFrequency = 1000.0;
        
        // Derived Constants
        public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
        public static final double drivingMotorFreeSpeedRps = NeoMotorConstants.freeSpeedRpm / 60;
        public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
            / drivingMotorReduction;
    }

    /**
     * Drive subsystem constants
     */
    public static final class DriveConstants {
        // Chassis Configuration
        public static final double trackWidth = Units.inchesToMeters(29);
        public static final double wheelBase = Units.inchesToMeters(29);
        
        // Performance Limits
        public static final double maxSpeedMetersPerSecond = 4.8;
        public static final double maxAngularSpeed = 2 * Math.PI;
        
        // Swerve Module Positions
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),    // Front Left
            new Translation2d(wheelBase / 2, -trackWidth / 2),   // Front Right
            new Translation2d(-wheelBase / 2, trackWidth / 2),   // Back Left
            new Translation2d(-wheelBase / 2, -trackWidth / 2)   // Back Right
        );

        // Module Angular Offsets (radians)
        public static final double frontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double frontRightChassisAngularOffset = 0;
        public static final double backLeftChassisAngularOffset = Math.PI;
        public static final double backRightChassisAngularOffset = Math.PI / 2;

        // CAN IDs and Encoder Ports by Module
        // Front Left Module
        public static final int frontLeftDrivingCanId = 11;
        public static final int frontLeftTurningCanId = 10;
        public static final int frontLeftEncoderPort = 0;
        
        // Front Right Module
        public static final int frontRightDrivingCanId = 15;
        public static final int frontRightTurningCanId = 14;
        public static final int frontRightEncoderPort = 1;
        
        // Back Left (Rear Left) Module
        public static final int rearLeftDrivingCanId = 13;
        public static final int rearLeftTurningCanId = 12;
        public static final int rearLeftEncoderPort = 2;
        
        // Back Right (Rear Right) Module
        public static final int rearRightDrivingCanId = 17;
        public static final int rearRightTurningCanId = 16;
        public static final int rearRightEncoderPort = 3;

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
}
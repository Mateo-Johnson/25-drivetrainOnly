package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utils.Constants.ModuleConstants;

public final class Config {
    public static final class MK4ISwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            //L3 MK4i Constants
            final double drivingMotorReduction = ModuleConstants.drivingMotorReduction; // L3 drive reduction
            final double wheelDiameterMeters = ModuleConstants.wheelDiameterMeters;
            final double driveWheelFreeSpeedRps = ModuleConstants.driveWheelFreeSpeedRps;

            //Calculate conversion factors
            double drivingFactor = wheelDiameterMeters * Math.PI / drivingMotorReduction;
            double drivingVelocityFeedForward = 1.0 / (driveWheelFreeSpeedRps * 
                wheelDiameterMeters * Math.PI);

            //Driving motor configuration
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
            
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);
            
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    //Tuned for L3
                    .pid(0.1, 0.0, 0.0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            //Turning motor configuration
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1.0, 0.0, 0.0) //Increased P gain for better holding
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Config;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {
    private final SparkMax drivingSpark;
    private final SparkMax turningSpark;

    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;
    
    private final AnalogEncoder analogEncoder;
    
    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    @SuppressWarnings("unused")
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    @SuppressWarnings("unused")
    private double lastAngle;
    private boolean hasBeenSeeded = false;
    private final Timer seedTimer = new Timer();
    private static final double seed_delay = 2.0; // Wait 2 seconds after boot

    // Constants for analog encoder configuration
    private static final double ENCODER_VOLTAGE_MIN = 0.01; // 1% minimum voltage
    private static final double ENCODER_VOLTAGE_MAX = 0.99; // 99% maximum voltage

    public Module(int drivingCANId, int turningCANId, int analogChannel, double cAngularOffset) {
        drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getEncoder();
        
        // Create and configure the AnalogEncoder
        // Using fullRange of 2π since we want radians, and expectedZero of 0
        analogEncoder = new AnalogEncoder(analogChannel, 2 * Math.PI, 0);
        // Set voltage range to avoid noisy readings at extremes
        analogEncoder.setVoltagePercentageRange(ENCODER_VOLTAGE_MIN, ENCODER_VOLTAGE_MAX);
        
        drivingClosedLoopController = drivingSpark.getClosedLoopController();
        turningClosedLoopController = turningSpark.getClosedLoopController();

        drivingSpark.configure(Config.MK4iSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        turningSpark.configure(Config.MK4iSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        chassisAngularOffset = cAngularOffset;
        
        // Start the seed timer
        seedTimer.start();
        
        // Initialize lastAngle but don't set the encoder position yet
        lastAngle = getAbsoluteAngle();
    }

    /**
     * Attempts to seed the relative encoder with the absolute encoder position.
     * Should be called periodically until successful.
     * @return true if seeding was successful, false if it needs to be called again
     */
    public boolean seedEncoderIfNeeded() {
        if (hasBeenSeeded) {
            return true;
        }

        // Check if we've waited long enough after boot
        if (seedTimer.get() < seed_delay) {
            return false;
        }

        // Get absolute angle and verify it's valid
        double absoluteAngle = getAbsoluteAngle();
        if (Double.isNaN(absoluteAngle)) {
            SmartDashboard.putString("Module " + turningSpark.getDeviceId() + " Seed Status", 
                "Invalid encoder reading");
            return false;
        }

        // Set the relative encoder position
        turningEncoder.setPosition(absoluteAngle);
        lastAngle = absoluteAngle;
        hasBeenSeeded = true;

        SmartDashboard.putString("Module " + turningSpark.getDeviceId() + " Seed Status", 
            "Seeded at angle: " + Math.toDegrees(absoluteAngle));
        return true;
    }

    /**
     * Gets the absolute angle in radians
     * AnalogEncoder.get() returns the value scaled by our fullRange (2π)
     * We then adjust by the chassis offset
     */
    private double getAbsoluteAngle() {
        // Get the raw angle in radians (since we set fullRange to 2π)
        double angle = analogEncoder.get();
        
        // Subtract the chassis offset
        angle -= chassisAngularOffset;
        
        // Put the angle in the correct range (-π to π)
        angle %= 2.0 * Math.PI;
        if (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        } else if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        
        return angle;
    }

    /**
     * Returns the current state of the module.
     * Uses the relative encoder for position if seeded, falls back to absolute if not.
     */
    public SwerveModuleState getState() {
        double currentAngleRadians = hasBeenSeeded ? 
            turningEncoder.getPosition() : getAbsoluteAngle();
        
        return new SwerveModuleState(
            drivingEncoder.getVelocity(),
            new Rotation2d(currentAngleRadians)
        );
    }

    /**
     * Returns the current position of the module.
     * Uses the relative encoder for position if seeded, falls back to absolute if not.
     */
    public SwerveModulePosition getPosition() {
        double currentAngleRadians = hasBeenSeeded ? 
            turningEncoder.getPosition() : getAbsoluteAngle();
            
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(currentAngleRadians)
        );
    }

    /**
     * Sets the desired state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Don't allow movement until encoders are seeded
        if (!hasBeenSeeded) {
            return;
        }

        double currentAngleRadians = turningEncoder.getPosition();
        
        @SuppressWarnings("deprecation")
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, 
            new Rotation2d(currentAngleRadians));

        drivingClosedLoopController.setReference(
            optimizedState.speedMetersPerSecond, 
            ControlType.kVelocity
        );
        
        double targetAngle = optimizedState.angle.getRadians();
        
        turningClosedLoopController.setReference(
            targetAngle, 
            ControlType.kPosition
        );

        this.desiredState = optimizedState;
        
        SmartDashboard.putNumber("Module " + turningSpark.getDeviceId() + " Absolute Angle", 
            Math.toDegrees(getAbsoluteAngle()));
        SmartDashboard.putNumber("Module " + turningSpark.getDeviceId() + " Relative Angle", 
            Math.toDegrees(currentAngleRadians));
        SmartDashboard.putNumber("Module " + turningSpark.getDeviceId() + " Target Angle", 
            Math.toDegrees(targetAngle));
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void close() {
        analogEncoder.close();
    }
}
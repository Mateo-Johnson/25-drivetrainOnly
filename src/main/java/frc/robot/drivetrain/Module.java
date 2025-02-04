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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Module {
    private final SparkMax drivingSpark;
    private final SparkMax turningSpark;

    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder turningEncoder;
    
    // Thrifty encoder setup using analog input
    private final AnalogInput analogEncoder;
    
    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    @SuppressWarnings("unused")
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    // Track the turning position
    private double lastAngle;

    public Module(int drivingCANId, int turningCANId, int analogChannel, double cAngularOffset) {
        drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        // Configure driving motor and encoder
        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getEncoder();
        
        // Configure Thrifty encoder as analog input
        analogEncoder = new AnalogInput(analogChannel);
        
        drivingClosedLoopController = drivingSpark.getClosedLoopController();
        turningClosedLoopController = turningSpark.getClosedLoopController();

        // Apply configurations to the SPARKs
        drivingSpark.configure(Config.MK4iSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        turningSpark.configure(Config.MK4iSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        chassisAngularOffset = cAngularOffset;
        
        // Initialize the turning encoder position based on absolute encoder
        lastAngle = getAbsoluteEncoder();
        turningEncoder.setPosition(lastAngle);
    }

    /**
     * Gets the absolute encoder angle in radians from the Thrifty encoder
     * The Thrifty encoder outputs 0-5V corresponding to 0-360 degrees
     */
    private double getAbsoluteEncoder() {
        // Get the raw voltage from the analog input
        double voltage = analogEncoder.getVoltage();
        
        // Convert voltage to angle (voltage/voltage_supply * 2π) to get radians
        double angle = voltage / RobotController.getVoltage5V() * 2.0 * Math.PI;
        
        // Apply the chassis offset
        angle -= chassisAngularOffset;
        
        // Put the angle in the correct range (-pi to pi)
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
     */
    public SwerveModuleState getState() {
        // Get the absolute angle for the current position
        double currentAngleRadians = getAbsoluteEncoder();
        
        return new SwerveModuleState(
            drivingEncoder.getVelocity(),
            new Rotation2d(currentAngleRadians)
        );
    }

    /**
     * Returns the current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(getAbsoluteEncoder())
        );
    }

    /**
     * Sets the desired state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Get the current angle
        double currentAngleRadians = getAbsoluteEncoder();
        
        // Optimize the reference state to avoid spinning further than 90 degrees
        @SuppressWarnings("deprecation")
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, 
            new Rotation2d(currentAngleRadians));

        // Command driving and turning motors
        drivingClosedLoopController.setReference(
            optimizedState.speedMetersPerSecond, 
            ControlType.kVelocity
        );
        
        // Calculate the new reference angle for position control
        double targetAngle = optimizedState.angle.getRadians();
        
        turningClosedLoopController.setReference(
            targetAngle, 
            ControlType.kPosition
        );

        // Save desired state
        this.desiredState = optimizedState;
        
        // Output debug values
        SmartDashboard.putNumber("Module Absolute Angle", Math.toDegrees(currentAngleRadians));
        SmartDashboard.putNumber("Module Target Angle", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Module Raw Voltage", analogEncoder.getVoltage());
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}
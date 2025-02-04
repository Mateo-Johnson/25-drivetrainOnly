package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Drivetrain extends SubsystemBase {
  // Create Swerve Modules
  private final Module frontLeft = new Module(
      DriveConstants.frontLeftDrivingCanId,
      DriveConstants.frontLeftTurningCanId,
      DriveConstants.frontLeftEncoderPort,
      DriveConstants.frontLeftChassisAngularOffset);

  private final Module frontRight = new Module(
      DriveConstants.frontRightDrivingCanId,
      DriveConstants.frontRightTurningCanId,
      DriveConstants.frontRightEncoderPort,
      DriveConstants.frontRightChassisAngularOffset);

  private final Module rearLeft = new Module(
      DriveConstants.rearLeftDrivingCanId,
      DriveConstants.rearLeftTurningCanId,
      DriveConstants.rearLeftEncoderPort,
      DriveConstants.backLeftChassisAngularOffset);

  private final Module rearRight = new Module(
      DriveConstants.rearRightDrivingCanId,
      DriveConstants.rearRightTurningCanId,
      DriveConstants.rearRightEncoderPort,
      DriveConstants.backRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry odometry;
  
  // Track initialization state
  private boolean modulesInitialized = false;
  private final Timer initTimer = new Timer();
  private static final double INIT_TIMEOUT_SECONDS = 5.0; // Maximum time to wait for initialization
  private boolean initTimeoutLogged = false;

  public Drivetrain() {
    // Start initialization timer
    initTimer.start();

    // Initialize odometry
    odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    // Load the RobotConfig from the GUI settings
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry, 
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
    );
  }

  @Override
  public void periodic() {
    // Handle initialization of swerve modules
    if (!modulesInitialized) {
      // Check if all modules are seeded
      boolean allSeeded = 
          frontLeft.seedEncoderIfNeeded() &&
          frontRight.seedEncoderIfNeeded() &&
          rearLeft.seedEncoderIfNeeded() &&
          rearRight.seedEncoderIfNeeded();

      if (allSeeded) {
        modulesInitialized = true;
        SmartDashboard.putString("Drive Status", "Initialized");
        initTimer.stop();
      } else if (initTimer.get() > INIT_TIMEOUT_SECONDS) {
        // Log timeout only once
        if (!initTimeoutLogged) {
          DriverStation.reportWarning("Swerve module initialization timed out!", false);
          initTimeoutLogged = true;
        }
        SmartDashboard.putString("Drive Status", "Init Timeout");
      } else {
        SmartDashboard.putString("Drive Status", "Initializing");
      }
    }

    // Update odometry if modules are initialized
    if (modulesInitialized) {
      odometry.update(
          Rotation2d.fromDegrees(gyro.getAngle()),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              rearLeft.getPosition(),
              rearRight.getPosition()
          });

      // Output telemetry data
      Pose2d pose = odometry.getPoseMeters();
      SmartDashboard.putNumber("Robot X", pose.getX());
      SmartDashboard.putNumber("Robot Y", pose.getY());
      SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
    }

    // Always update module telemetry regardless of initialization state
    updateModuleTelemetry();
  }

  /**
   * Updates SmartDashboard with module telemetry data
   */
  private void updateModuleTelemetry() {
    String[] moduleNames = {"FL", "FR", "RL", "RR"};
    Module[] modules = {frontLeft, frontRight, rearLeft, rearRight};

    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState state = modules[i].getState();
      SmartDashboard.putNumber(moduleNames[i] + " Speed", state.speedMetersPerSecond);
      SmartDashboard.putNumber(moduleNames[i] + " Angle", state.angle.getDegrees());
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState());
  }

  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

    /**
     * The main drive method for the robot. Use this anytime you want to make the robot rotate,
     * translate, or any combination of the two.
     * @param
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.maxSpeedMetersPerSecond;
    double rotDelivered = rotation * DriveConstants.maxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

    /**
     * Set the swerve drivetrain wheels into an X formation
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

    /**
     * Resets the NavX to 0 position
   */
  public void zeroHeading() {
    gyro.reset();
  }

    /**
     * Method to get the current heading from the gyro as a Double
     * @return The current heading (in degrees)
     * 
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }
}
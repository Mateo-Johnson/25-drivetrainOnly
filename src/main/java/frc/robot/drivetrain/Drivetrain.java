package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class Drivetrain extends SubsystemBase {
  private final Module frontLeft;
  private final Module frontRight;
  private final Module rearLeft;
  private final Module rearRight;
  private final AHRS gyro;
  private SwerveDriveOdometry odometry;
  private volatile boolean modulesInitialized = false;
  
  public Drivetrain() {
      // Create modules
      frontLeft = new Module(
          DriveConstants.frontLeftDrivingCanId,
          DriveConstants.frontLeftTurningCanId,
          DriveConstants.frontLeftEncoderPort,
          DriveConstants.frontLeftChassisAngularOffset
      );
      frontRight = new Module(
          DriveConstants.frontRightDrivingCanId,
          DriveConstants.frontRightTurningCanId,
          DriveConstants.frontRightEncoderPort,
          DriveConstants.frontRightChassisAngularOffset
      );
      rearLeft = new Module(
          DriveConstants.rearLeftDrivingCanId,
          DriveConstants.rearLeftTurningCanId,
          DriveConstants.rearLeftEncoderPort,
          DriveConstants.rearLeftChassisAngularOffset
      );
      rearRight = new Module(
          DriveConstants.rearRightDrivingCanId,
          DriveConstants.rearRightTurningCanId,
          DriveConstants.rearRightEncoderPort,
          DriveConstants.rearRightChassisAngularOffset
      );
      
      // Initialize gyro
      gyro = new AHRS(NavXComType.kMXP_SPI);
      
      // Start initialization thread
      Thread initThread = new Thread(() -> {
          try {
              Thread.sleep(2000); // 2 second delay
              odometry = new SwerveDriveOdometry(
                  DriveConstants.kDriveKinematics,
                  Rotation2d.fromDegrees(gyro.getAngle()),
                  new SwerveModulePosition[] {
                      frontLeft.getPosition(),
                      frontRight.getPosition(),
                      rearLeft.getPosition(),
                      rearRight.getPosition()
                  }
              );
              modulesInitialized = true;
          } catch (InterruptedException e) {
              Thread.currentThread().interrupt();
          }
      });
      initThread.start();
      
      // Configure AutoBuilder
      try {
          RobotConfig config = RobotConfig.fromGUISettings();
          AutoBuilder.configure(
              this::getPose,
              this::resetOdometry,
              this::getRobotRelativeSpeeds,
              (speeds, feedforwards) -> driveRobotRelative(speeds),
              new PPHolonomicDriveController(
                  new PIDConstants(5.0, 0.0, 0.0),
                  new PIDConstants(5.0, 0.0, 0.0)
              ),
              config,
              () -> {
                  var alliance = DriverStation.getAlliance();
                  return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
              },
              this
          );
      } catch (Exception e) {
          e.printStackTrace();
      }
  }
  
  public boolean areModulesInitialized() {
      return modulesInitialized;
  }
  
  private ChassisSpeeds getRobotRelativeSpeeds() {
      if (!areModulesInitialized()) {
          return new ChassisSpeeds(0, 0, 0);
      }
      return DriveConstants.kDriveKinematics.toChassisSpeeds(
          frontLeft.getState(),
          frontRight.getState(),
          rearLeft.getState(),
          rearRight.getState()
      );
  }
  
  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
      if (!areModulesInitialized()) {
          return;
      }
      SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      setModuleStates(states);
  }
  
  @Override
  public void periodic() {
      if (!areModulesInitialized()) {
          return;
      }
      odometry.update(
          Rotation2d.fromDegrees(gyro.getAngle()),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              rearLeft.getPosition(),
              rearRight.getPosition()
          }
      );
      SmartDashboard.putNumber("FL", frontLeft.getRawAngle());
      SmartDashboard.putNumber("FR", frontRight.getRawAngle());
      SmartDashboard.putNumber("BL", rearLeft.getRawAngle());
      SmartDashboard.putNumber("BR", rearRight.getRawAngle());
  }
  
  public Pose2d getPose() {
      if (!areModulesInitialized()) {
          return new Pose2d();
      }
      return odometry.getPoseMeters();
  }
  
  public void resetOdometry(Pose2d pose) {
      if (!areModulesInitialized()) {
          return;
      }
      odometry.resetPosition(
          Rotation2d.fromDegrees(gyro.getAngle()),
          new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              rearLeft.getPosition(),
              rearRight.getPosition()
          },
          pose
      );
  }
  
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
      if (!areModulesInitialized()) {
          return;
      }
      double xSpeedDelivered = xSpeed * DriveConstants.maxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeed * DriveConstants.maxSpeedMetersPerSecond;
      double rotDelivered = rotation * DriveConstants.maxAngularSpeed;
      
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeedDelivered, 
              ySpeedDelivered, 
              rotDelivered,
              Rotation2d.fromDegrees(gyro.getAngle())
          )
          : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
      );
      
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, 
          DriveConstants.maxSpeedMetersPerSecond
      );
      
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      rearLeft.setDesiredState(swerveModuleStates[2]);
      rearRight.setDesiredState(swerveModuleStates[3]);
  }
  
  public void setX() {
      if (!areModulesInitialized()) {
          return;
      }
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
      if (!areModulesInitialized()) {
          return;
      }
      SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, 
          DriveConstants.maxSpeedMetersPerSecond
      );
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      rearLeft.setDesiredState(desiredStates[2]);
      rearRight.setDesiredState(desiredStates[3]);
  }
  
  public void resetEncoders() {
      if (!areModulesInitialized()) {
          return;
      }
      frontLeft.resetEncoders();
      rearLeft.resetEncoders();
      frontRight.resetEncoders();
      rearRight.resetEncoders();
  }
  
  public void zeroHeading() {
      gyro.reset();
  }
  
  public double getHeading() {
      return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }
  
  public double getTurnRate() {
      return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }
}
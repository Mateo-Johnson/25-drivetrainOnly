package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.Config;
import edu.wpi.first.math.MathUtil;

public class Module {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AnalogEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public Module(int drivingCANId, int turningCANId, int turningEncoderChannel, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = new AnalogEncoder(turningEncoderChannel);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    m_drivingSpark.configure(Config.MK4ISwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSpark.configure(Config.MK4ISwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;

    // Read absolute encoder position, apply offset, and set the turning motor position accordingly
    double absoluteAngle = getEncoderRadians() - m_chassisAngularOffset;
    m_turningClosedLoopController.setReference(absoluteAngle, ControlType.kPosition);
    
    m_desiredState.angle = new Rotation2d(absoluteAngle);
    m_drivingEncoder.setPosition(0);
}

  private double getEncoderRadians() {
    return MathUtil.inputModulus(m_turningEncoder.get() * 2.0 * Math.PI, 0.0, 2.0 * Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(getEncoderRadians() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getEncoderRadians() - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(getEncoderRadians()));

    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    m_turningClosedLoopController.setReference(getEncoderRadians() - m_chassisAngularOffset, ControlType.kPosition);
}



  public double getRawAngle() {
    return getEncoderRadians();
  }

  public double getRawPosition() {
    return m_turningEncoder.get();
  }
}

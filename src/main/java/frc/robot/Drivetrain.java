// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters

  // IO interfaces
  private final DriveSideIO leftDriveIO;
  private final DriveSideIO rightDriveIO;
  private final GyroIO gyroIO;

  // IO inputs
  private DriveSideIOInputsAutoLogged leftDriveIOInputs = new DriveSideIOInputsAutoLogged();
  private DriveSideIOInputsAutoLogged rightDriveIOInputs = new DriveSideIOInputsAutoLogged();
  private GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the
   * gyro.
   */
  public Drivetrain(DriveSideIO left, DriveSideIO right, GyroIO gyro) {
    leftDriveIO = left;
    rightDriveIO = right;
    gyroIO = gyro;

    gyroIO.reset();

    leftDriveIO.updateInputs(leftDriveIOInputs);
    rightDriveIO.updateInputs(rightDriveIOInputs);
    gyroIO.updateInputs(gyroIOInputs);

    var wheelPositions = getDifferentialDrivePositions();

    m_odometry = new DifferentialDriveOdometry(
        gyroIOInputs.yawPosition, wheelPositions.leftMeters, wheelPositions.rightMeters);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftAngularVelocity = RadiansPerSecond.of(speeds.leftMetersPerSecond / kWheelRadius);
    var rightAngularVelocity = RadiansPerSecond.of(speeds.rightMetersPerSecond / kWheelRadius);
    leftDriveIO.setDriveVelocity(leftAngularVelocity);
    rightDriveIO.setDriveVelocity(rightAngularVelocity);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public DifferentialDriveWheelPositions getDifferentialDrivePositions() {
    var leftPositionMeters = leftDriveIOInputs.position.in(Radians) * kWheelRadius;
    var rightPositionMeters = rightDriveIOInputs.position.in(Radians) * kWheelRadius;
    return new DifferentialDriveWheelPositions(leftPositionMeters, rightPositionMeters);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    var wheelPositions = getDifferentialDrivePositions();
    m_odometry.update(
        gyroIOInputs.yawPosition, wheelPositions.leftMeters, wheelPositions.rightMeters);
  }
}

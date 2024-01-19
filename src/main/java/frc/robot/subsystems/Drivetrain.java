// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 0.07; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final DifferentialDriveOdometry m_odometry;

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.141);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final MutableMeasure<Voltage> leftVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Voltage> rightVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Distance> leftDistance = MutableMeasure.mutable(Units.Meters.of(0));
  private final MutableMeasure<Distance> rightDistance = MutableMeasure.mutable(Units.Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> leftVelocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));
  private final MutableMeasure<Velocity<Distance>> rightVelocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

  private final PIDController leftPID = new PIDController(0, 0, 0);
  private final PIDController rightPID = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 0, 0);
  private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0, 0, 0);

  private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

  private final SysIdRoutine sysID = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
    (volt) -> {
      tankDrive(
        volt.in(Units.Volts) / RobotController.getBatteryVoltage(),
        volt.in(Units.Volts) / RobotController.getBatteryVoltage()
      );
    },
    (log) -> {
      log.motor("left-drive")
        .voltage(leftVoltage.mut_replace(m_leftMotor.get() * RobotController.getBatteryVoltage(), Units.Volts))
        .linearPosition(leftDistance.mut_replace(getLeftDistanceMeter(), Units.Meters))
        .linearVelocity(leftVelocity.mut_replace(getLeftVelocityMetersPerSecond(), Units.MetersPerSecond));
      
        log.motor("right-drive")
        .voltage(rightVoltage.mut_replace(m_rightMotor.get() * RobotController.getBatteryVoltage(), Units.Volts))
        .linearPosition(rightDistance.mut_replace(getRightDistanceMeter(), Units.Meters))
        .linearVelocity(rightVelocity.mut_replace(getRightVelocityMetersPerSecond(), Units.MetersPerSecond));
    },
    this)
  );

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

    AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double left, double right) {
    m_diffDrive.tankDrive(left, right);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocityMetersPerSecond() {
    return m_leftEncoder.getRate();
  }

  public double getRightVelocityMetersPerSecond() {
    return m_rightEncoder.getRate();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    DifferentialDriveWheelSpeeds speeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

    double leftVoltage = leftFF.calculate(speeds.leftMetersPerSecond) + leftPID.calculate(getLeftVelocityMetersPerSecond(), speeds.leftMetersPerSecond);
    double rightVoltage = rightFF.calculate(speeds.rightMetersPerSecond) + rightPID.calculate(getRightVelocityMetersPerSecond(), speeds.rightMetersPerSecond);

    tankDrive(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / RobotController.getBatteryVoltage());
  }

  public Command getQuasistatic(Direction dir) {
    return sysID.quasistatic(dir);
  }

  public Command getDynamic(Direction dir) {
    return sysID.dynamic(dir);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(getGyroAngle(), getLeftDistanceMeter(), getRightDistanceMeter(), pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond()));
  }

  public void drive(ChassisSpeeds speeds) {
    targetChassisSpeeds = speeds;
  }

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(getGyroAngleZ());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_turret = new TalonFX(26, kCANBus);

  private final CANcoder m_turretcc = new CANcoder(36, kCANBus);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private static final double kRotorToSensorRatio = 4.0;
  private static final double kSensorToMechanismRatio = 162.0 / 23.0;

  private final StatusSignal<Boolean> f_fusedSensorOutOfSync =
      m_turret.getFault_FusedSensorOutOfSync(false);
  private final StatusSignal<Boolean> sf_fusedSensorOutOfSync =
      m_turret.getStickyFault_FusedSensorOutOfSync(false);
  private final StatusSignal<Boolean> f_remoteSensorInvalid =
      m_turret.getFault_RemoteSensorDataInvalid(false);
  private final StatusSignal<Boolean> sf_remoteSensorInvalid =
      m_turret.getStickyFault_RemoteSensorDataInvalid(false);

  private final StatusSignal<Angle> fx_pos = m_turret.getPosition(false);
  private final StatusSignal<AngularVelocity> fx_vel = m_turret.getVelocity(false);
  private final StatusSignal<Angle> cc_absPos = m_turretcc.getAbsolutePosition(false);
  private final StatusSignal<Angle> cc_pos = m_turretcc.getPosition(false);
  private final StatusSignal<AngularVelocity> cc_vel = m_turretcc.getVelocity(false);
  private final StatusSignal<Angle> fx_rotorPos = m_turret.getRotorPosition(false);

  // CW and CCW are viewed from top of robot, turret 0 degrees will be facing rear of robot
  // (opposite of intake)
  private final double maxrotationCW = 225;
  private final double maxrotationCCW = -225;

  private double turretdegrees = 0;
  private double m_targetAngleDeg = 0.0;
  private double m_lastCommandedRotations = 0.0;
  private double m_startupSeedAngleDeg = 0.0;

  public Turret() {

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Rotations.of(.459 - .084 + .0125 + .011));
    m_turretcc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Feedback.FeedbackRemoteSensorID = m_turretcc.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.SensorToMechanismRatio = kSensorToMechanismRatio;
    cfg.Feedback.RotorToSensorRatio = kRotorToSensorRatio;

    // /* Configure gear ratio */
    // FeedbackConfigs fdb = cfg.Feedback;
    // fdb.SensorToMechanismRatio = 28.17391304; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(4)) // 4 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(
                10)) // 10 Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(300)); // 300

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 75; // A position error of 0.2 rotations results in 12 V output was 55
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_turret.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    double startupAngleDeg = estimateStartupAngleFromAbsoluteCancoderDegrees();
    zeroTurretDegrees(startupAngleDeg);
    System.out.println("Turret seeded from CANcoder absolute angle: " + startupAngleDeg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretdegrees = getCurrentTurretAngleDegrees();
    SmartDashboard.putNumber("Calc Turret Degrees", turretdegrees);
    SmartDashboard.putNumber("Turret/AngleErrorDeg", getAngleErrorDegrees());
    SmartDashboard.putNumber("Turret/CurrentPositionRot", fx_pos.getValueAsDouble());
    SmartDashboard.putNumber("Turret/CurrentVelocityRps", fx_vel.getValueAsDouble());
    SmartDashboard.putNumber("Turret/TargetAngleDeg", m_targetAngleDeg);
    SmartDashboard.putNumber("Turret/TargetPositionRot", m_lastCommandedRotations);
    SmartDashboard.putNumber("Turret/StartupSeedAngleDeg", m_startupSeedAngleDeg);
    SmartDashboard.putNumber("Turret/Motor Position: ", fx_pos.getValueAsDouble());
    SmartDashboard.putNumber("Turret/Motor Velocity: ", fx_vel.getValueAsDouble());
    SmartDashboard.putNumber("Turret/Rotor Position: ", fx_rotorPos.getValueAsDouble());
    SmartDashboard.putNumber("Turret/Cancoder Abs Position: ", cc_absPos.getValueAsDouble());
    SmartDashboard.putNumber("Turret/Cancoder Position: ", cc_pos.getValueAsDouble());
    SmartDashboard.putNumber("Turret/Cancoder Velocity: ", cc_vel.getValueAsDouble());
    SmartDashboard.putBoolean("Turret/FusedSensorOutOfSync", f_fusedSensorOutOfSync.getValue());
    SmartDashboard.putBoolean(
        "Turret/StickyFusedSensorOutOfSync", sf_fusedSensorOutOfSync.getValue());
    SmartDashboard.putBoolean("Turret/RemoteSensorInvalid", f_remoteSensorInvalid.getValue());
    SmartDashboard.putBoolean(
        "Turret/StickyRemoteSensorInvalid", sf_remoteSensorInvalid.getValue());
  }

  public void manright() {
    m_turret.set(-.075);
  }

  public void manleft() {
    m_turret.set(.075);
  }

  public void stop() {
    m_turret.set(0);
  }

  public void setturrettoangle(double angle) {
    double currentAngleDeg = getCurrentTurretAngleDegrees();
    double targetAngleDeg = chooseShortestPathTarget(currentAngleDeg, angle);
    double targetRotations = degreesToMotorRotations(targetAngleDeg);
    m_targetAngleDeg = targetAngleDeg;
    m_lastCommandedRotations = targetRotations;
    m_turret.setControl(m_mmReq.withPosition(targetRotations).withSlot(0));
    SmartDashboard.putNumber("Turret/TargetAngleDeg", targetAngleDeg);
  }

  public double getCurrentTurretAngleDegrees() {
    return motorRotationsToDegrees(m_turret.getPosition().getValueAsDouble());
  }

  public void zeroTurretDegrees(double knownAngleDeg) {
    double clampedAngleDeg = clampToTurretLimits(knownAngleDeg);
    m_turret.setPosition(degreesToMotorRotations(clampedAngleDeg));
    turretdegrees = clampedAngleDeg;
    m_targetAngleDeg = clampedAngleDeg;
    m_lastCommandedRotations = degreesToMotorRotations(clampedAngleDeg);
    m_startupSeedAngleDeg = clampedAngleDeg;
  }

  public double getAngleErrorDegrees() {
    return m_targetAngleDeg - getCurrentTurretAngleDegrees();
  }

  public boolean isAtTarget(double toleranceDeg) {
    return Math.abs(getAngleErrorDegrees()) <= toleranceDeg;
  }

  private double chooseShortestPathTarget(double currentAngleDeg, double requestedAngleDeg) {
    double[] candidates = {requestedAngleDeg, requestedAngleDeg + 360.0, requestedAngleDeg - 360.0};
    double bestTarget = clampToTurretLimits(requestedAngleDeg);
    double bestDelta = Math.abs(bestTarget - currentAngleDeg);

    for (double candidate : candidates) {
      if (candidate < maxrotationCCW || candidate > maxrotationCW) {
        continue;
      }
      double delta = Math.abs(candidate - currentAngleDeg);
      if (delta < bestDelta) {
        bestDelta = delta;
        bestTarget = candidate;
      }
    }
    return bestTarget;
  }

  private double clampToTurretLimits(double angleDeg) {
    if (angleDeg > maxrotationCW) {
      return maxrotationCW;
    }
    if (angleDeg < maxrotationCCW) {
      return maxrotationCCW;
    }
    return angleDeg;
  }

  private double estimateStartupAngleFromAbsoluteCancoderDegrees() {
    double absoluteSensorRotations = m_turretcc.getAbsolutePosition().getValueAsDouble();
    double[] sensorCandidates = {
      absoluteSensorRotations - 1.0, absoluteSensorRotations, absoluteSensorRotations + 1.0
    };

    double bestAngleDeg = sensorCandidates[0] * 360.0 / kSensorToMechanismRatio;
    double bestDistanceFromForward = Math.abs(bestAngleDeg);

    for (double sensorCandidate : sensorCandidates) {
      double candidateAngleDeg = sensorCandidate * 360.0 / kSensorToMechanismRatio;
      double distanceFromForward = Math.abs(candidateAngleDeg);
      if (distanceFromForward < bestDistanceFromForward) {
        bestDistanceFromForward = distanceFromForward;
        bestAngleDeg = candidateAngleDeg;
      }
    }

    SmartDashboard.putNumber("Turret/StartupAbsSensorRot", absoluteSensorRotations);
    SmartDashboard.putNumber("Turret/StartupEstimatedAngleDeg", bestAngleDeg);
    return bestAngleDeg;
  }

  private double degreesToMotorRotations(double angleDeg) {
    return angleDeg / 360.0;
  }

  private double motorRotationsToDegrees(double motorRotations) {
    return motorRotations * 360.0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.*;
import static frc.robot.generated.TunerConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final String kOscillationFrequencyHzKey = "Intake/OscillationFrequencyHz";

  /** Creates a new Intake. */
  private final TalonFX m_spinner = new TalonFX(28, kCANBus);

  private final TalonFX m_extender = new TalonFX(22, kCANBus);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  private final Timer m_oscillationTimer = new Timer();

  private boolean m_oscillating = false;

  static boolean intakehomed;

  public Intake() {
    TalonFXConfiguration extendercfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = extendercfg.Feedback;
    fdb.SensorToMechanismRatio = 9.217;

    /* Configure Motion Magic */
    MotionMagicConfigs mm = extendercfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(100)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(200)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(500));
    Slot0Configs slot0 = extendercfg.Slot0;

    slot0.kS = 0.5; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    TalonFXConfiguration spinnerCfg = new TalonFXConfiguration();

    spinnerCfg.CurrentLimits.SupplyCurrentLimit = 35;
    spinnerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    spinnerCfg.CurrentLimits.StatorCurrentLimit = 35;
    spinnerCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_extender.getConfigurator().apply(extendercfg);
      status = m_spinner.getConfigurator().apply(spinnerCfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    intakehomed = false;
    SmartDashboard.putNumber(kOscillationFrequencyHzKey, intakeOscillationFrequencyHz);
    SmartDashboard.putBoolean("Intake/Oscillating", false);
  }

  @Override
  public void periodic() {
    if (m_oscillating) {
      double frequencyHz =
          Math.max(
              0.0,
              SmartDashboard.getNumber(kOscillationFrequencyHzKey, intakeOscillationFrequencyHz));
      double phase = (m_oscillationTimer.get() * frequencyHz) % 1.0;
      if (frequencyHz > 0.0 && phase < 0.5) {
        setExtenderPosition(intakedeployposition);
      } else {
        setExtenderPosition(intakemidstopposition);
      }
    }

    SmartDashboard.putNumber("Spinner Temp", m_spinner.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Spinner TCurrent", m_spinner.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("Intake/Oscillating", m_oscillating);
  }

  public void runintake() {
    m_spinner.set(-.70);
  }

  public void stopintake() {
    m_spinner.set(0);
  }

  // manual controls
  public void manualintakedeploy() {
    disableOscillation();
    m_extender.set(.3);
  }

  public void manualintakeretract() {
    disableOscillation();
    m_extender.set(-.3);
  }

  public void manualstopdeploy() {
    disableOscillation();
    m_extender.set(0);
  }

  public void deployintake() {
    disableOscillation();
    setExtenderPosition(intakedeployposition);
  }

  public void midstopintake() {
    disableOscillation();
    setExtenderPosition(intakemidstopposition);
  }

  public void retractintake() {
    disableOscillation();
    setExtenderPosition(intakeretractposition);
  }

  public void startOscillation() {
    m_oscillating = true;
    m_oscillationTimer.restart();
  }

  public void stopOscillation() {
    disableOscillation();
    setExtenderPosition(intakedeployposition);
  }

  // homing commands

  public void resetencoder() {
    m_extender.setPosition(0);
  }

  public void setintakepower(double power) {
    disableOscillation();
    m_extender.set(power);
  }

  public void setintakestop() {
    disableOscillation();
    m_extender.set(0);
  }

  private void disableOscillation() {
    m_oscillating = false;
    m_oscillationTimer.stop();
  }

  private void setExtenderPosition(double position) {
    m_extender.setControl(m_mmReq.withPosition(position).withSlot(0));
  }
}

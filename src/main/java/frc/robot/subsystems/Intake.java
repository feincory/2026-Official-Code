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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX m_spinner = new TalonFX(28, kCANBus);

  private final TalonFX m_extender = new TalonFX(22, kCANBus);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  static boolean intakehomed;

  public Intake() {
    TalonFXConfiguration extendercfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = extendercfg.Feedback;
    fdb.SensorToMechanismRatio = 10.86956521;

    /* Configure Motion Magic */
    MotionMagicConfigs mm = extendercfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(75)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(150)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(300));
    Slot0Configs slot0 = extendercfg.Slot0;

    slot0.kS = 0.5; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 50; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_extender.getConfigurator().apply(extendercfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    intakehomed = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runintake() {
    m_spinner.set(-.90);
  }

  public void stopintake() {
    m_spinner.set(0);
  }

  // manual controls
  public void manualintakedeploy() {
    m_extender.set(.3);
  }

  public void manualintakeretract() {
    m_extender.set(-.3);
  }

  public void manualstopdeploy() {
    m_extender.set(0);
  }

  public void deployintake() {
    setExtenderPosition(intakedeployposition);
  }

  public void midstopintake() {
    setExtenderPosition(intakemidstopposition);
  }

  public void retractintake() {
    setExtenderPosition(intakeretractposition);
  }

  // homing commands

  public void resetencoder() {
    m_extender.setPosition(0);
  }

  public void setintakepower(double power) {
    m_extender.set(power);
  }

  public void setintakestop() {
    m_extender.set(0);
  }

  private void setExtenderPosition(double position) {
    m_extender.setControl(m_mmReq.withPosition(position).withSlot(0));
  }
}

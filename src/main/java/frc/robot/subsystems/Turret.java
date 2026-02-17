// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_turret = new TalonFX(26, kCANBus);

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private final AnalogInput m_pot = new AnalogInput(0);

  private final int potmaxvalue = 3949;
  private final int potminvalue = 400;
  // middle of pot travel will be 2027
  private final double gearboxreuction = 162 / 23;
  // CW and CCW are viewed from top of robot, turret 0 degrees will be facing rear of robot
  // (opposite of intake)
  private final double maxrotationCW = 225;
  private final double maxrotationCCW = -225;

  private final double turretrange = maxrotationCW - maxrotationCCW;
  private final double potrange = potmaxvalue - potminvalue;

  private final double potdegreesratio = -turretrange / potrange;

  private final double potmiddlerange = (potrange / 2) + potminvalue;

  private double turretdegrees = 0;

  public Turret() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 28.17391304; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(25)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(25)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(300));

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 03; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 30; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 1.5; // A velocity error of 1 rps results in 0.5 V output

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_turret.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    turretdegrees = (m_pot.getValue() - potmiddlerange) * potdegreesratio;
    m_turret.setPosition(turretdegrees / 360);
    System.out.println("Set Turret Angle" + turretdegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pot Value", m_pot.getValue());

    turretdegrees = (m_pot.getValue() - potmiddlerange) * potdegreesratio;
    SmartDashboard.putNumber("Calc Turret Degrees", turretdegrees);
  }

  public void manright() {
    m_turret.set(-.2);
  }

  public void manleft() {
    m_turret.set(.2);
  }

  public void stop() {
    m_turret.set(0);
  }

  public void setturrettoangle(double angle) {
    m_turret.setControl(m_mmReq.withPosition(angle / 360).withSlot(0));
  }
}

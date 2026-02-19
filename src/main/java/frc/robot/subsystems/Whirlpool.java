// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Whirlpool extends SubsystemBase {
  /** Creates a new Whirlpool. */
  private final TalonFX m_whirlpool = new TalonFX(27, kCANBus);

  private final TalonFX m_feeder = new TalonFX(24, kCANBus);

  public Whirlpool() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startwhirlpool() {
    m_whirlpool.set(.6);
    m_feeder.set(1);
  }

  public void stopwhirlpool() {
    m_whirlpool.set(0);
    m_feeder.set(0);
  }

  public void reversewhirlpool() {
    m_whirlpool.set(-.4);
    m_feeder.set(-.4);
  }

  public void startfeeder() {
    m_feeder.set(0);
  }

  public void stopfeeder() {
    m_feeder.set(0);
  }
}

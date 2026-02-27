// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX m_climber = new TalonFX(40, kCANBus);

  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startclimber() {
    m_climber.set(1);
  }

  public void reverseclimber() {
    m_climber.set(-1);
  }

  public void stopclimber() {
    m_climber.set(0);
  }
}

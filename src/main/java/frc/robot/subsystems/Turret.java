// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_turret = new TalonFX(26, kCANBus);

  private final AnalogInput m_pot = new AnalogInput(0);

  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Pot Value", m_pot.getValue());
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
}

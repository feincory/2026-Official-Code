// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Whirlpool extends SubsystemBase {
  /** Creates a new Whirlpool. */
  private final TalonFX m_whirlpool = new TalonFX(27, kCANBus);

  private final TalonFX m_feeder = new TalonFX(24, kCANBus);

  public Whirlpool() {
    TalonFXConfiguration whirlpoolconfig = new TalonFXConfiguration();
    whirlpoolconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .5;
    whirlpoolconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_whirlpool.getConfigurator().apply(whirlpoolconfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

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
    m_whirlpool.set(-1);
    m_feeder.set(-1);
  }

  public void startfeeder() {
    m_feeder.set(0);
  }

  public void stopfeeder() {
    m_feeder.set(0);
  }
}

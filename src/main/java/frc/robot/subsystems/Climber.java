// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX m_climber = new TalonFX(40, kCANBus);

  public Climber() {
    m_climber.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration climberconfig = new TalonFXConfiguration();
    climberconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .75;
    climberconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_climber.getConfigurator().apply(climberconfig);
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

  public void startclimber() {
    m_climber.set(.5);
  }

  public void reverseclimber() {
    m_climber.set(-.5);
  }

  public void stopclimber() {
    m_climber.set(0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.generated.TunerConstants.kCANBus;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Whirlpool extends SubsystemBase {
  /** Creates a new Whirlpool. */
  private final TalonFX m_whirlpool = new TalonFX(27, kCANBus);

  private final TalonFXS m_floor = new TalonFXS(45, kCANBus);

  private final TalonFX m_feeder = new TalonFX(24, kCANBus);

  public Whirlpool() {
    SmartDashboard.putNumber("Whirlpool Speed", 1.0);
    SmartDashboard.putNumber("Floor Speed", .7);

    TalonFXConfiguration whirlpoolconfig = new TalonFXConfiguration();
    whirlpoolconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    whirlpoolconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .3;
    whirlpoolconfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = .3;
    whirlpoolconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5;
    whirlpoolconfig.CurrentLimits.StatorCurrentLimit = 25;
    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      status = m_whirlpool.getConfigurator().apply(whirlpoolconfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    TalonFXSConfiguration floorconfig = new TalonFXSConfiguration();
    floorconfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    floorconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .3;
    floorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .3;
    floorconfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = .3;
    floorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5;
    floorconfig.CurrentLimits.StatorCurrentLimit = 30;

    StatusCode floorstatus = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      floorstatus = m_floor.getConfigurator().apply(floorconfig);
      if (floorstatus.isOK()) break;
    }
    if (!floorstatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + floorstatus.toString());
    }

    TalonFXConfiguration feederconfConfig = new TalonFXConfiguration();
    feederconfConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .1;
    feederconfConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .1;
    feederconfConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = .1;
    feederconfConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .1;
    feederconfConfig.CurrentLimits.StatorCurrentLimit = 30;
    StatusCode feederstatus = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; ++i) {
      feederstatus = m_feeder.getConfigurator().apply(feederconfConfig);
      if (feederstatus.isOK()) break;
    }
    if (!feederstatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + feederstatus.toString());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startwhirlpool() {
    m_whirlpool.set(SmartDashboard.getNumber("Whirlpool Speed", 1.0));
    m_floor.set(SmartDashboard.getNumber("Floor Speed", .7));
    // m_whirlpool.set(1.0);
    m_feeder.set(.8);
  }

  public void stopwhirlpool() {
    m_whirlpool.set(0);
    m_floor.set(0);
    m_feeder.set(0);
  }

  public void reversewhirlpool() {
    m_whirlpool.set(-.5);
    m_floor.set(-.5);
    m_feeder.set(-.5);
  }

  public void startfeeder() {
    m_feeder.set(0);
  }

  public void stopfeeder() {
    m_feeder.set(0);
  }
}

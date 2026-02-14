// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Volts;
import static frc.robot.generated.TunerConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX m_shooter = new TalonFX(36, "rio");

  private final TalonFX m_accelmtr = new TalonFX(25, kCANBus);
  private final TalonFX m_hoodtiltmtr = new TalonFX(35, "rio");

  NeutralOut m_coastmode = new NeutralOut();

  public Shooter() {

    // Shooter motor configuration
    TalonFXConfiguration shooterconfigs = new TalonFXConfiguration();
    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    shooterconfigs.Slot0.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    shooterconfigs.Slot0.kP = 5; // An error of 1 rotation per second results in 5 A output
    shooterconfigs.Slot0.kI = 0; // No output for integrated error
    shooterconfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 40 A
    shooterconfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));
    shooterconfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
    shooterconfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    // Accelerator motor configuration
    TalonFXConfiguration accelconfigs = new TalonFXConfiguration();
    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    accelconfigs.Slot0.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    accelconfigs.Slot0.kP = 5; // An error of 1 rotation per second results in 5 A output
    accelconfigs.Slot0.kI = 0; // No output for integrated error
    accelconfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 40 A
    accelconfigs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        .withPeakReverseTorqueCurrent(Amps.of(-40));
    accelconfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
    accelconfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    TalonFXConfiguration hoodtiltconfig = new TalonFXConfiguration();
    hoodtiltconfig.Slot0.kP = 60; // An error of 1 rotation results in 60 A output
    hoodtiltconfig.Slot0.kI = 0; // No output for integrated error
    hoodtiltconfig.Slot0.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    hoodtiltconfig.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
        .withPeakReverseTorqueCurrent(Amps.of(-120));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_shooter.getConfigurator().apply(shooterconfigs);
      status = m_accelmtr.getConfigurator().apply(accelconfigs);
      status = m_hoodtiltmtr.getConfigurator().apply(hoodtiltconfig);
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

  public void runshooter() {
    m_shooter.set(-.45);
    m_accelmtr.set(.45);
  }

  public void stopshooter() {
    m_shooter.set(0);
    m_accelmtr.set(0);
  }

  public void shootersetvelocity(double rpm) {
    m_shooter.set(rpm);
    m_accelmtr.set(rpm);
  }

  // homing commands

  public void resetencoder() {
    m_hoodtiltmtr.setPosition(0);
  }

  public void setshooterhoodpower(double power) {
    m_hoodtiltmtr.set(power);
  }

  public void shooterhoodup() {
    m_hoodtiltmtr.set(.1);
  }

  public void shooterhooddown() {
    m_hoodtiltmtr.set(-.1);
  }

  public void stopshooterhood() {
    m_hoodtiltmtr.set(0);
  }
}

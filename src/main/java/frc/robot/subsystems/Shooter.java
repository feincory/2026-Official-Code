// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.Volts;
import static frc.robot.generated.TunerConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final TalonFX m_shooter = new TalonFX(36, "rio");

  private final TalonFX m_accelmtr = new TalonFX(25, kCANBus);
  private final TalonFX m_hoodtiltmtr = new TalonFX(35, "rio");
  private final MotionMagicVoltage mmhood = new MotionMagicVoltage(0);
  private final VelocityVoltage shooterVelocityReq = new VelocityVoltage(0);
  private final VelocityVoltage accelVelocityReq = new VelocityVoltage(0);
  // private static final double kAccelRpsMultiplier = -1.0;

  NeutralOut m_coastmode = new NeutralOut();
  NeutralOut m_brakemode = new NeutralOut();
  // private double hoodmin = 0;
  // private double hoodmax = 14.185;

  public Shooter() {

    // Shooter motor configuration
    TalonFXConfiguration shooterconfigs = new TalonFXConfiguration();
    shooterconfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    shooterconfigs.Slot0.kS = 0.5; // To account for friction, add 0.1 V of static feedforward
    shooterconfigs.Slot0.kV =
        0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts
    // / rotation per second
    shooterconfigs.Slot0.kP = 0.75; // An error of 1 rotation per second results in 0.11 V output
    shooterconfigs.Slot0.kI = 0; // No output for integrated error
    shooterconfigs.Slot0.kD = 0.2; // No output for error derivative
    // Peak output of 8 volts
    shooterconfigs.Voltage.withPeakForwardVoltage(Volts.of(12))
        .withPeakReverseVoltage(Volts.of(-12));

    shooterconfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = .2;
    shooterconfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .2;

    // Accelerator motor configuration
    TalonFXConfiguration accelconfigs = new TalonFXConfiguration();
    accelconfigs.Slot0.kS = 0.15; // To account for friction, add 0.1 V of static feedforward
    accelconfigs.Slot0.kV =
        0.122; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts
    // / rotation per second
    accelconfigs.Slot0.kP = 1.5; // An error of 1 rotation per second results in 0.11 V output
    accelconfigs.Slot0.kI = 0; // No output for integrated error
    accelconfigs.Slot0.kD = 0.1; // No output for error derivative
    // Peak output of 8 volts
    accelconfigs.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

    accelconfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = .3;
    accelconfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .3;

    // hood tilt motor configs
    TalonFXConfiguration hoodtiltconfig = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = hoodtiltconfig.Feedback;
    hoodtiltconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    fdb.SensorToMechanismRatio = 14.185; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = hoodtiltconfig.MotionMagic;
    mm.withMotionMagicCruiseVelocity(
            RotationsPerSecond.of(15)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(100)) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(1000));

    Slot0Configs slot0 = hoodtiltconfig.Slot0;
    slot0.kS = 1; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.3; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 120; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.2; // A velocity error of 1 rps results in 0.5 V output

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

    SmartDashboard.putNumber("Shooter RPS", 0);
    SmartDashboard.putNumber("Hood Position", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void runshooter() {
    m_shooter.set(.45);
    m_accelmtr.set(-.45);
  }

  public void stopshooter() {
    m_shooter.set(0);
    m_accelmtr.set(0);
  }

  public void shootersetvelocity(double rps) {
    m_shooter.setControl(shooterVelocityReq.withVelocity(rps));
    m_accelmtr.setControl(accelVelocityReq.withVelocity(rps * 1.2));
  }

  public void setshooterhood(double position) {
    m_hoodtiltmtr.setControl(mmhood.withPosition(position).withSlot(0));
  }

  public void resethood() {
    m_hoodtiltmtr.setPosition(0);
    System.out.println("hood zeroed");
  }

  public void stophood() {
    m_hoodtiltmtr.set(0);
    System.out.println("hood zeroed");
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

  public double getShooterRps() {
    return m_shooter.getVelocity().getValueAsDouble();
  }

  public boolean isAtSpeed(double targetRps, double toleranceRps) {
    return Math.abs(Math.abs(getShooterRps()) - Math.abs(targetRps)) <= toleranceRps;
  }
}

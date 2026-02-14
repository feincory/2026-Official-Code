// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.khoodhomeswt;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeShooterHood extends Command {
  /** Creates a new HomeIntake. */
  private final Shooter shooter;

  private final DigitalInput homeSwitch;
  private static final double HOMING_SPEED = 0.1; // Slow descent
  private static final double TIMEOUT = 5.0; // Stop after 3 seconds
  private Timer timer = new Timer();

  public HomeShooterHood(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.homeSwitch = new DigitalInput(khoodhomeswt);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (homeSwitch.get()) { // Switch is triggered (active low)
      shooter.setshooterhoodpower(0);
      shooter.resetencoder(); // Set position to zero
    } else {
      shooter.setshooterhoodpower(HOMING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setshooterhoodpower(0);

    if (timer.hasElapsed(TIMEOUT)) {
      System.out.println("ERROR: Intake homing failed! Sensor not detected.");
    } else {
      System.out.println("Homing Success");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return homeSwitch.get() || timer.hasElapsed(TIMEOUT);
  }
}

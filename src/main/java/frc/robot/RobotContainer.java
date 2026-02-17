// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Whirlpool;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.ShooterCalc;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter = new Shooter();
  private final Whirlpool whirlpool = new Whirlpool();
  private final Climber climber = new Climber();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final ShooterCalc shooterCalc = new ShooterCalc();
  private final Vision vision;

  // Controller
  private final CommandJoystick flightcontroller = new CommandJoystick(0);
  private final CommandXboxController operatorcontroller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Run Intake", new InstantCommand(intake::runintake));

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> flightcontroller.getRawAxis(1),
            () -> -flightcontroller.getRawAxis(0),
            () -> -flightcontroller.getRawAxis(3)));
    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    flightcontroller.button(16).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Shooter Controls
    operatorcontroller.y().onTrue(new InstantCommand(shooter::runshooter));
    operatorcontroller.y().onFalse(new InstantCommand(shooter::stopshooter));
    // Shooter Controls
    operatorcontroller.a().onTrue(new InstantCommand(whirlpool::startwhirlpool));
    operatorcontroller.a().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    operatorcontroller.b().onTrue(new InstantCommand(whirlpool::reversewhirlpool));
    operatorcontroller.b().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    operatorcontroller.povUp().onTrue(new InstantCommand(shooter::shooterhoodup));
    operatorcontroller.povUp().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.povDown().onTrue(new InstantCommand(shooter::shooterhooddown));
    operatorcontroller.povDown().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.start().onTrue(new InstantCommand(() -> shooter.setshooterhood(.75)));
    operatorcontroller.start().onFalse(new InstantCommand(() -> shooter.setshooterhood(.25)));

    operatorcontroller.back().onTrue(new InstantCommand(() -> shooter.resethood()));
    operatorcontroller.back().onTrue(new InstantCommand(() -> shooter.stophood()));

    // turret controls
    operatorcontroller.povRight().onTrue(new InstantCommand(turret::manleft));
    operatorcontroller.povRight().onFalse(new InstantCommand(turret::stop));

    operatorcontroller.povLeft().onTrue(new InstantCommand(turret::manright));
    operatorcontroller.povLeft().onFalse(new InstantCommand(turret::stop));

    operatorcontroller.leftTrigger().onTrue(new InstantCommand(() -> turret.setturrettoangle(45)));
    operatorcontroller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> turret.setturrettoangle(-45)));
    // Intake controls
    operatorcontroller.rightBumper().onTrue(new InstantCommand(intake::manualintakedeploy));
    operatorcontroller.rightBumper().onFalse(new InstantCommand(intake::manualstopdeploy));
    operatorcontroller.leftBumper().onTrue(new InstantCommand(intake::manualintakeretract));
    operatorcontroller.leftBumper().onFalse(new InstantCommand(intake::manualstopdeploy));
    operatorcontroller.leftStick().onTrue(new InstantCommand(intake::runintake));
    operatorcontroller.leftStick().onFalse(new InstantCommand(intake::stopintake));

    flightcontroller.button(13).onTrue(new InstantCommand(intake::runintake));
    flightcontroller.button(13).onFalse(new InstantCommand(intake::stopintake));
    // operatorcontroller
    //     .leftTrigger()
    //     .onTrue(
    //         Commands.sequence(
    //             new InstantCommand(whirlpool::startfeeder),
    //             new WaitCommand(2),
    //             new InstantCommand(whirlpool::startwhirlpool)));

    // operatorcontroller.leftTrigger().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    // operatorcontroller
    //     .povDown()
    //     .onTrue(Command.runonce(shooterCalc.getDistanceToGoal, ShooterCalc));

    // Reset gyro to 0° when B button is pressed
    flightcontroller
        .button(14)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

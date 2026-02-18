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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeShooterHood;
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
import java.util.NavigableMap;
import java.util.TreeMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final String kAimOffsetInchesKey = "AutoAim/LateralOffsetInches";
  private static final String kDistanceOffsetInchesKey = "AutoAim/DistanceOffsetInches";

  // Subsystems
  private final Drive drive;
  private final Shooter shooter = new Shooter();
  private final Whirlpool whirlpool = new Whirlpool();
  private final Climber climber = new Climber();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  // +X forward, +Y left. Replace with measured turret center offset from robot center.
  private static final Translation2d kTurretOffsetFromRobotCenterMeters =
      new Translation2d(-0.0889, .1461);
  //   new Translation2d(-0.0889, .1461);
  // Turret 0 deg in your Turret subsystem points to rear of robot.
  private static final Rotation2d kTurretZeroDirectionInRobotFrame = Rotation2d.fromDegrees(180.0);
  // Replace with real field goal center positions for your game.
  // private static final Translation2d kBlueGoalCenterFieldMeters = new Translation2d(4.629,
  // 4.032);
  private static final Translation2d kBlueGoalCenterFieldMeters = new Translation2d(11.918, 4.03);
  private static final Translation2d kRedGoalCenterFieldMeters = new Translation2d(11.918, 4.03);
  // private static final Translation2d kRedGoalCenterFieldMeters = new Translation2d(11.918, 4.03);

  private final ShooterCalc shooterCalc =
      new ShooterCalc(
          kTurretOffsetFromRobotCenterMeters,
          kTurretZeroDirectionInRobotFrame,
          true,
          -225.0,
          225.0,
          buildShooterRpsTable(),
          buildHoodPercentTable());
  private final Vision vision;

  // Controller
  private final CommandJoystick flightcontroller = new CommandJoystick(0);
  private final CommandXboxController operatorcontroller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Run Intake", new InstantCommand(intake::runintake));
    SmartDashboard.putNumber(kAimOffsetInchesKey, -20);
    SmartDashboard.putNumber(kDistanceOffsetInchesKey, 0.0);

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

    operatorcontroller.x().onTrue(new InstantCommand(() -> shooter.shootersetvelocity(30)));
    operatorcontroller.x().onFalse(new InstantCommand(shooter::stopshooter));

    operatorcontroller.a().onTrue(new InstantCommand(whirlpool::startwhirlpool));
    operatorcontroller.a().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    flightcontroller.button(6).onTrue(new InstantCommand(whirlpool::startwhirlpool));
    flightcontroller.button(6).onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    operatorcontroller.b().onTrue(new InstantCommand(whirlpool::reversewhirlpool));

    operatorcontroller.b().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    operatorcontroller.povUp().onTrue(new InstantCommand(shooter::shooterhoodup));
    operatorcontroller.povUp().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.povDown().onTrue(new InstantCommand(shooter::shooterhooddown));
    operatorcontroller.povDown().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.start().onTrue(new InstantCommand(() -> shooter.setshooterhood(.75)));
    operatorcontroller.start().onFalse(new InstantCommand(() -> shooter.setshooterhood(.25)));

    operatorcontroller.back().onTrue(new HomeShooterHood(shooter));
    // operatorcontroller.back().onTrue(new InstantCommand(() -> shooter.stophood()));

    // turret controls
    operatorcontroller.povRight().onTrue(new InstantCommand(turret::manleft));
    operatorcontroller.povRight().onFalse(new InstantCommand(turret::stop));

    operatorcontroller.povLeft().onTrue(new InstantCommand(turret::manright));
    operatorcontroller.povLeft().onFalse(new InstantCommand(turret::stop));

    // operatorcontroller.leftTrigger().onTrue(new InstantCommand(() ->
    // turret.setturrettoangle(45)));
    // operatorcontroller
    //     .rightTrigger()
    //     .onTrue(new InstantCommand(() -> turret.setturrettoangle(-45)));

    // Hold left trigger to continuously auto-aim turret using pose + turret offset geometry.
    flightcontroller
        .button(4)
        .whileTrue(
            Commands.run(
                () -> {
                  double lateralOffsetInches = SmartDashboard.getNumber(kAimOffsetInchesKey, 0.0);
                  shooterCalc.setLateralAimOffsetMeters(Units.inchesToMeters(lateralOffsetInches));
                  double distanceOffsetInches =
                      SmartDashboard.getNumber(kDistanceOffsetInchesKey, 0.0);
                  shooterCalc.setDistanceOffsetMeters(Units.inchesToMeters(distanceOffsetInches));

                  Translation2d goalCenter = getCurrentAllianceGoalCenter();
                  ShooterCalc.ShotSolution shot =
                      shooterCalc.calculateShot(drive.getPose(), goalCenter);

                  turret.setturrettoangle(shot.getTurretCommandDegrees());
                  shooter.shootersetvelocity(shot.getShooterRps());
                  shooter.setshooterhood(shot.getHoodPercent());

                  SmartDashboard.putNumber("AutoAim/DistanceMeters", shot.getDistanceMeters());
                  SmartDashboard.putNumber(
                      "AutoAim/EffectiveDistanceMeters", shot.getEffectiveDistanceMeters());
                  SmartDashboard.putNumber("AutoAim/TurretCmdDeg", shot.getTurretCommandDegrees());
                  SmartDashboard.putNumber("AutoAim/ShooterRps", shot.getShooterRps());
                  SmartDashboard.putNumber("AutoAim/HoodPercent", shot.getHoodPercent());
                  SmartDashboard.putNumber(
                      "AutoAim/LateralOffsetMeters", shooterCalc.getLateralAimOffsetMeters());
                  SmartDashboard.putNumber(
                      "AutoAim/DistanceOffsetMeters", shooterCalc.getDistanceOffsetMeters());
                },
                turret,
                shooter));

    flightcontroller.button(4).onFalse(new InstantCommand(shooter::stopshooter));
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

  private Translation2d getCurrentAllianceGoalCenter() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red
        ? kRedGoalCenterFieldMeters
        : kBlueGoalCenterFieldMeters;
  }

  private static NavigableMap<Double, Double> buildShooterRpsTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(0.889, 28.0);
    table.put(1.2954, 30.0);
    table.put(1.778, 30.0);
    table.put(2.54, 32.0);
    table.put(2.794, 35.0);
    table.put(3.556, 37.0);
    table.put(4.704, 40.0);
    table.put(4.7244, 43.0);
    table.put(5.3848, 43.0);
    table.put(7.1374, 50.0);
    table.put(9.9568, 75.0);
    return table;
  }

  private static NavigableMap<Double, Double> buildHoodPercentTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(0.889, 0.0);
    table.put(1.2954, 0.05);
    table.put(1.778, 0.25);
    table.put(2.54, 0.3);
    table.put(2.794, 0.25);
    table.put(3.556, 0.3);
    table.put(4.4704, 0.37);
    table.put(4.7244, 0.38);
    table.put(5.3848, 0.48);
    table.put(7.1374, 0.51);
    table.put(9.9568, 1.0);
    return table;
  }
}

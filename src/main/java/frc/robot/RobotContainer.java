// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.MovingPassShotConstants;
import frc.robot.Constants.MovingShotConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HomeShooterHood;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
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
  private static final double kAimOffsetAdjustStepInches = 4.0;
  private static final double kAimOffsetStickThreshold = 0.9;
  private static final double hoodpassingaddition = .2;

  // Subsystems
  private final Drive drive;
  private final Shooter shooter = new Shooter();
  private final Whirlpool whirlpool = new Whirlpool();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();
  private final LEDLights ledLights = new LEDLights();
  private final Vision vision;
  // +X forward, +Y left. Replace with measured turret center offset from robot center.
  private static final Translation2d kTurretOffsetFromRobotCenterMeters =
      new Translation2d(-0.0889, .1461);
  //   new Translation2d(-0.0889, .1461);
  // Turret 0 deg in your Turret subsystem points to rear of robot.
  private static final Rotation2d kTurretZeroDirectionInRobotFrame = Rotation2d.fromDegrees(180.0);
  // Replace with real field goal center positions for your game.
  private static final Translation2d kBlueGoalCenterFieldMeters = new Translation2d(4.629, 4.03);
  private static final Translation2d kRedGoalCenterFieldMeters = new Translation2d(11.918, 4.03);

  private final ShooterCalc shooterCalc =
      new ShooterCalc(
          kTurretOffsetFromRobotCenterMeters,
          kTurretZeroDirectionInRobotFrame,
          true,
          -225.0,
          225.0,
          buildShooterRpsTable(),
          buildHoodPercentTable(),
          buildShooterPassing(),
          buildShooterPassinghood());

  // Controller
  private final CommandJoystick flightcontroller = new CommandJoystick(0);
  private final CommandXboxController operatorcontroller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Run Intake", new InstantCommand(intake::runintake));
    NamedCommands.registerCommand("Run Whirlpool", new InstantCommand(whirlpool::startwhirlpool));
    NamedCommands.registerCommand("Stop Whirlpool", new InstantCommand(whirlpool::stopwhirlpool));
    NamedCommands.registerCommand("Stop Shooter", new InstantCommand(shooter::stopshooter));

    NamedCommands.registerCommand("Deploy Intake", new InstantCommand(intake::deployintake));
    NamedCommands.registerCommand("Osc Intake", new InstantCommand(intake::startOscillation));
    NamedCommands.registerCommand(
        "Auto Shoot", (Commands.run(() -> runAutoAim(false), turret, shooter, whirlpool)));
    NamedCommands.registerCommand(
        "Moving Shot", (Commands.run(() -> runMovingShot(true), turret, shooter, whirlpool)));

    seedDashboardDefaults();

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
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation));
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
    intake.setDriveSpeedMetersPerSecSupplier(drive::getLinearSpeedMetersPerSec);

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
            () -> flightcontroller.getRawAxis(1) * 1.1,
            () -> -flightcontroller.getRawAxis(0) * 1.1,
            () -> -flightcontroller.getRawAxis(3),
            this::getMovingShotDriveLinearScale,
            this::getMovingShotDriveOmegaScale));
    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,[\]\[]

    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    flightcontroller.button(16).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Shooter Controls
    operatorcontroller.y().onTrue(new InstantCommand(shooter::runshooter));
    operatorcontroller.y().onFalse(new InstantCommand(shooter::stopshooter));

    operatorcontroller
        .x()
        .onTrue(
            new InstantCommand(
                () -> shooter.setshooterhood(SmartDashboard.getNumber("Hood Position", 0))));
    operatorcontroller.x().onFalse(new InstantCommand(shooter::stopshooter));

    operatorcontroller
        .x()
        .onTrue(
            new InstantCommand(
                () -> shooter.shootersetvelocity(SmartDashboard.getNumber("Shooter RPS", 30.0))));
    operatorcontroller.x().onFalse(new InstantCommand(shooter::stopshooter));

    // operatorcontroller.a().onTrue(new InstantCommand(whirlpool::startwhirlpool));
    // operatorcontroller.a().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    flightcontroller.button(7).onTrue(new InstantCommand(shooter::reverseshooter));
    flightcontroller.button(7).onFalse(new InstantCommand(shooter::stopshooter));

    operatorcontroller.povUp().onTrue(new InstantCommand(shooter::shooterhoodup));
    operatorcontroller.povUp().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.povDown().onTrue(new InstantCommand(shooter::shooterhooddown));
    operatorcontroller.povDown().onFalse(new InstantCommand(shooter::stopshooterhood));

    operatorcontroller.start().onTrue(new InstantCommand(() -> shooter.setshooterhood(.75)));
    operatorcontroller.start().onFalse(new InstantCommand(() -> shooter.setshooterhood(.25)));

    operatorcontroller.back().onTrue(new HomeShooterHood(shooter));

    // whirlpool controls
    flightcontroller.button(13).onTrue(new InstantCommand(whirlpool::startwhirlpool));
    flightcontroller.button(13).onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    flightcontroller.button(7).onTrue(new InstantCommand(whirlpool::reversewhirlpool));
    flightcontroller.button(7).onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    flightcontroller.button(7).onTrue(new InstantCommand(intake::reverseintake));
    flightcontroller.button(7).onFalse(new InstantCommand(intake::stopintake));

    operatorcontroller.b().onTrue(new InstantCommand(whirlpool::startwhirlpool));
    operatorcontroller.b().onFalse(new InstantCommand(whirlpool::stopwhirlpool));

    // turret controls
    operatorcontroller.povRight().onTrue(new InstantCommand(turret::manleft));
    operatorcontroller.povRight().onFalse(new InstantCommand(turret::stop));

    operatorcontroller.povLeft().onTrue(new InstantCommand(turret::manright));
    operatorcontroller.povLeft().onFalse(new InstantCommand(turret::stop));

    new Trigger(() -> operatorcontroller.getLeftY() <= -kAimOffsetStickThreshold)
        .onTrue(new InstantCommand(() -> adjustDistanceOffsetsInches(kAimOffsetAdjustStepInches)));

    new Trigger(() -> operatorcontroller.getLeftY() >= kAimOffsetStickThreshold)
        .onTrue(new InstantCommand(() -> adjustDistanceOffsetsInches(-kAimOffsetAdjustStepInches)));
    new Trigger(() -> operatorcontroller.getLeftX() >= kAimOffsetStickThreshold)
        .onTrue(new InstantCommand(() -> adjustLateralOffsetsInches(-kAimOffsetAdjustStepInches)));
    new Trigger(() -> operatorcontroller.getLeftX() <= -kAimOffsetStickThreshold)
        .onTrue(new InstantCommand(() -> adjustLateralOffsetsInches(kAimOffsetAdjustStepInches)));

    // Hold button 4 for normal auto-aim (turret + shooter + hood only).
    flightcontroller.button(4).whileTrue(Commands.run(() -> runAutoAim(false), turret, shooter));

    // Hold button 5 for auto-aim + auto-feed when ready.
    flightcontroller
        .button(5)
        .whileTrue(Commands.run(() -> runAutoAim(true), turret, shooter, whirlpool));

    // for 2nd driver to trigger shooter
    operatorcontroller
        .a()
        .whileTrue(Commands.run(() -> runAutoAim(true), turret, shooter, whirlpool));

    flightcontroller
        .button(12)
        .whileTrue(Commands.run(() -> runMovingShot(true), turret, shooter, whirlpool));

    flightcontroller.button(4).onFalse(new InstantCommand(shooter::stopshooter));
    flightcontroller
        .button(5)
        .onFalse(
            Commands.sequence(
                new InstantCommand(shooter::stopshooter),
                new InstantCommand(whirlpool::stopwhirlpool)));
    operatorcontroller
        .a()
        .onFalse(
            Commands.sequence(
                new InstantCommand(shooter::stopshooter),
                new InstantCommand(whirlpool::stopwhirlpool)));
    flightcontroller
        .button(12)
        .onFalse(
            Commands.sequence(
                new InstantCommand(shooter::stopshooter),
                new InstantCommand(whirlpool::stopwhirlpool)));
    // Intake controls

    operatorcontroller.rightBumper().onTrue(new InstantCommand(intake::manualintakedeploy));
    operatorcontroller.rightBumper().onFalse(new InstantCommand(intake::manualstopdeploy));
    operatorcontroller.leftBumper().onTrue(new InstantCommand(intake::manualintakeretract));
    operatorcontroller.leftBumper().onFalse(new InstantCommand(intake::manualstopdeploy));
    // flightcontroller.button(15).onTrue(new InstantCommand(intake::resetencoder));
    flightcontroller.button(9).onTrue(new InstantCommand(intake::deployintake));
    flightcontroller.button(8).onTrue(new InstantCommand(intake::retractintake));

    // flightcontroller.button(8).onFalse(new InstantCommand(intake::midstopintake));

    // flightcontroller.button(9).onFalse(new InstantCommand(intake::midstopintake));
    // flightcontroller.button(8).onFalse(new InstantCommand(intake::runintake));
    // flightcontroller.button(9).onFalse(new InstantCommand(intake::runintake));
    flightcontroller.button(9).onTrue(new InstantCommand(intake::runintake));
    flightcontroller.button(8).onTrue(new InstantCommand(intake::runintake));
    flightcontroller.button(3).onTrue(new InstantCommand(intake::startOscillation));
    flightcontroller.button(3).onTrue(new InstantCommand(intake::runintake));
    flightcontroller.button(3).onFalse(new InstantCommand(intake::stopOscillation));
    flightcontroller.button(15).onTrue(new InstantCommand(intake::resetencoder));

    flightcontroller.button(2).onTrue(new InstantCommand(intake::runintake));
    flightcontroller.button(2).onFalse(new InstantCommand(intake::stopintake));

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

  public Command getDisableStopCommand() {
    return Commands.sequence(
            new InstantCommand(shooter::stopshooter),
            new InstantCommand(whirlpool::stopwhirlpool),
            new InstantCommand(intake::stopintake))
        .ignoringDisable(true);
  }

  public void onTeleopInit() {
    ledLights.onTeleopInit();
  }

  public void onAutoExitToTeleop() {
    intake.exitAutoToTeleop();
  }

  private void seedDashboardDefaults() {
    SmartDashboard.putNumber(
        AutoAimConstants.LATERAL_OFFSET_INCHES_KEY, AutoAimConstants.LATERAL_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.DISTANCE_OFFSET_INCHES_KEY,
        AutoAimConstants.DISTANCE_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.TURRET_ROTATION_LEAD_SECONDS_KEY,
        AutoAimConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.TURRET_ROTATION_COMP_SCALE_KEY,
        AutoAimConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.TURRET_READY_TOLERANCE_DEG_KEY,
        AutoAimConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.SHOOTER_READY_TOLERANCE_RPS_KEY,
        AutoAimConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY,
        AutoAimConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.ZONE_SPLIT_Y_METERS_KEY, AutoAimConstants.ZONE_SPLIT_Y_METERS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.OWN_ZONE_MAX_X_METERS_KEY, AutoAimConstants.OWN_ZONE_MAX_X_METERS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.PASS_NO_SHOOT_CENTER_Y_KEY,
        AutoAimConstants.PASS_NO_SHOOT_CENTER_Y_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.PASS_NO_SHOOT_WIDTH_METERS_KEY,
        AutoAimConstants.PASS_NO_SHOOT_WIDTH_METERS_DEFAULT);
    SmartDashboard.putNumber(
        AutoAimConstants.BLUE_PASS_BOTTOM_X_KEY,
        AutoAimConstants.BLUE_PASS_TARGET_BOTTOM_DEFAULT_FIELD_METERS.getX());
    SmartDashboard.putNumber(
        AutoAimConstants.BLUE_PASS_BOTTOM_Y_KEY,
        AutoAimConstants.BLUE_PASS_TARGET_BOTTOM_DEFAULT_FIELD_METERS.getY());
    SmartDashboard.putNumber(
        AutoAimConstants.BLUE_PASS_TOP_X_KEY,
        AutoAimConstants.BLUE_PASS_TARGET_TOP_DEFAULT_FIELD_METERS.getX());
    SmartDashboard.putNumber(
        AutoAimConstants.BLUE_PASS_TOP_Y_KEY,
        AutoAimConstants.BLUE_PASS_TARGET_TOP_DEFAULT_FIELD_METERS.getY());

    SmartDashboard.putNumber(
        MovingShotConstants.LATERAL_OFFSET_INCHES_KEY,
        MovingShotConstants.LATERAL_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.DISTANCE_OFFSET_INCHES_KEY,
        MovingShotConstants.DISTANCE_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.TURRET_ROTATION_LEAD_SECONDS_KEY,
        MovingShotConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.TURRET_ROTATION_COMP_SCALE_KEY,
        MovingShotConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.TURRET_READY_TOLERANCE_DEG_KEY,
        MovingShotConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.SHOOTER_READY_TOLERANCE_RPS_KEY,
        MovingShotConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY,
        MovingShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.DRIVER_MAX_LINEAR_SCALE_KEY,
        MovingShotConstants.DRIVER_MAX_LINEAR_SCALE_DEFAULT);
    SmartDashboard.putNumber(
        MovingShotConstants.DRIVER_MAX_OMEGA_SCALE_KEY,
        MovingShotConstants.DRIVER_MAX_OMEGA_SCALE_DEFAULT);
    SmartDashboard.putBoolean(
        MovingShotConstants.USE_MOTION_COMPENSATION_KEY,
        MovingShotConstants.USE_MOTION_COMPENSATION_DEFAULT);

    SmartDashboard.putNumber(
        MovingPassShotConstants.LATERAL_OFFSET_INCHES_KEY,
        MovingPassShotConstants.LATERAL_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.DISTANCE_OFFSET_INCHES_KEY,
        MovingPassShotConstants.DISTANCE_OFFSET_INCHES_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.TURRET_ROTATION_LEAD_SECONDS_KEY,
        MovingPassShotConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.TURRET_ROTATION_COMP_SCALE_KEY,
        MovingPassShotConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.TURRET_READY_TOLERANCE_DEG_KEY,
        MovingPassShotConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.SHOOTER_READY_TOLERANCE_RPS_KEY,
        MovingPassShotConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY,
        MovingPassShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.DRIVER_MAX_LINEAR_SCALE_KEY,
        MovingPassShotConstants.DRIVER_MAX_LINEAR_SCALE_DEFAULT);
    SmartDashboard.putNumber(
        MovingPassShotConstants.DRIVER_MAX_OMEGA_SCALE_KEY,
        MovingPassShotConstants.DRIVER_MAX_OMEGA_SCALE_DEFAULT);
    SmartDashboard.putBoolean(
        MovingPassShotConstants.USE_MOTION_COMPENSATION_KEY,
        MovingPassShotConstants.USE_MOTION_COMPENSATION_DEFAULT);
  }

  private void runAutoAim(boolean autoShootEnabled) {
    double lateralOffsetInches =
        SmartDashboard.getNumber(
            AutoAimConstants.LATERAL_OFFSET_INCHES_KEY,
            AutoAimConstants.LATERAL_OFFSET_INCHES_DEFAULT);
    shooterCalc.setLateralAimOffsetMeters(Units.inchesToMeters(lateralOffsetInches));
    double distanceOffsetInches =
        SmartDashboard.getNumber(
            AutoAimConstants.DISTANCE_OFFSET_INCHES_KEY,
            AutoAimConstants.DISTANCE_OFFSET_INCHES_DEFAULT);
    shooterCalc.setDistanceOffsetMeters(Units.inchesToMeters(distanceOffsetInches));

    Pose2d robotPose = drive.getPose();
    Translation2d targetCenter = getCurrentAllianceAimTarget(robotPose);
    boolean isPassingShot = !isRobotInOwnZone(robotPose);
    boolean inPassNoShootZone = isRobotInPassNoShootZone(robotPose);
    ShooterCalc.ShotSolution shot =
        shooterCalc.calculateShot(robotPose, targetCenter, isPassingShot);

    double yawRateRadPerSec = drive.getYawRateRadPerSec();
    double leadSec =
        SmartDashboard.getNumber(
            AutoAimConstants.TURRET_ROTATION_LEAD_SECONDS_KEY,
            AutoAimConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT);
    double compScale =
        SmartDashboard.getNumber(
            AutoAimConstants.TURRET_ROTATION_COMP_SCALE_KEY,
            AutoAimConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT);
    double turretRotationCompDeg = Units.radiansToDegrees(yawRateRadPerSec * leadSec * compScale);

    turret.setturrettoangle(shot.getTurretCommandDegrees() + turretRotationCompDeg);
    shooter.shootersetvelocity(shot.getShooterRps());
    shooter.setshooterhood(shot.getHoodPercent());

    double turretReadyToleranceDeg =
        SmartDashboard.getNumber(
            AutoAimConstants.TURRET_READY_TOLERANCE_DEG_KEY,
            AutoAimConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT);
    double shooterReadyToleranceRps =
        SmartDashboard.getNumber(
            AutoAimConstants.SHOOTER_READY_TOLERANCE_RPS_KEY,
            AutoAimConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT);
    double autoShootMaxRobotSpeedMps =
        SmartDashboard.getNumber(
            AutoAimConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY,
            AutoAimConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT);
    boolean turretReady = turret.isAtTarget(turretReadyToleranceDeg);
    boolean shooterReady = shooter.isAtSpeed(shot.getShooterRps(), shooterReadyToleranceRps);
    double robotLinearSpeedMps = drive.getLinearSpeedMetersPerSec();
    boolean robotSlowEnough = robotLinearSpeedMps <= autoShootMaxRobotSpeedMps;
    boolean movementGateSatisfied = isPassingShot || robotSlowEnough;
    boolean passZoneGateSatisfied = !isPassingShot || !inPassNoShootZone;
    boolean canAutoFeed =
        autoShootEnabled
            && turretReady
            && shooterReady
            && movementGateSatisfied
            && passZoneGateSatisfied;
    if (autoShootEnabled) {
      if (canAutoFeed) {
        whirlpool.startwhirlpool();
      } else {
        whirlpool.stopwhirlpool();
      }
    }

    SmartDashboard.putNumber("AutoAim/TargetX", targetCenter.getX());
    SmartDashboard.putNumber("AutoAim/TargetY", targetCenter.getY());
    SmartDashboard.putNumber("AutoAim/DistanceMeters", shot.getDistanceMeters());
    SmartDashboard.putNumber("AutoAim/EffectiveDistanceMeters", shot.getEffectiveDistanceMeters());
    SmartDashboard.putNumber("AutoAim/TurretCmdDeg", shot.getTurretCommandDegrees());
    SmartDashboard.putNumber("AutoAim/ShooterRps", shot.getShooterRps());
    SmartDashboard.putNumber("AutoAim/HoodPercent", shot.getHoodPercent());
    SmartDashboard.putNumber(
        "AutoAim/LateralOffsetMeters", shooterCalc.getLateralAimOffsetMeters());
    SmartDashboard.putNumber("AutoAim/DistanceOffsetMeters", shooterCalc.getDistanceOffsetMeters());
    SmartDashboard.putNumber("AutoAim/YawRateRadPerSec", yawRateRadPerSec);
    SmartDashboard.putNumber("AutoAim/RobotLinearSpeedMps", robotLinearSpeedMps);
    SmartDashboard.putNumber("AutoAim/TurretRotationCompDeg", turretRotationCompDeg);
    SmartDashboard.putBoolean("AutoAim/TurretReady", turretReady);
    SmartDashboard.putBoolean("AutoAim/ShooterReady", shooterReady);
    SmartDashboard.putBoolean("AutoAim/RobotSlowEnough", robotSlowEnough);
    SmartDashboard.putBoolean("AutoAim/IsPassingShot", isPassingShot);
    SmartDashboard.putBoolean("AutoAim/InPassNoShootZone", inPassNoShootZone);
    SmartDashboard.putBoolean("AutoAim/CanAutoFeed", canAutoFeed);
  }

  private void runMovingShot(boolean autoShootEnabled) {
    Pose2d robotPose = drive.getPose();
    boolean isPassingShot = !isRobotInOwnZone(robotPose);

    double lateralOffsetInches =
        SmartDashboard.getNumber(
            getMovingShotLateralOffsetKey(isPassingShot),
            getMovingShotLateralOffsetDefault(isPassingShot));
    shooterCalc.setLateralAimOffsetMeters(Units.inchesToMeters(lateralOffsetInches));
    double distanceOffsetInches =
        SmartDashboard.getNumber(
            getMovingShotDistanceOffsetKey(isPassingShot),
            getMovingShotDistanceOffsetDefault(isPassingShot));
    shooterCalc.setDistanceOffsetMeters(Units.inchesToMeters(distanceOffsetInches));

    Translation2d targetCenter = getCurrentAllianceAimTarget(robotPose);
    Translation2d compensatedTarget = targetCenter;
    Translation2d fieldVelocity = drive.getFieldRelativeVelocityMetersPerSec();
    boolean useMotionCompensation =
        SmartDashboard.getBoolean(
            getMovingShotUseMotionCompensationKey(isPassingShot),
            getMovingShotUseMotionCompensationDefault(isPassingShot));
    double targetLeadLookupDistanceMeters =
        shooterCalc.calculateShot(robotPose, targetCenter, isPassingShot).getDistanceMeters();
    double targetLeadSec =
        interpolateLookup(
            targetLeadLookupDistanceMeters, getMovingShotTargetLeadTable(isPassingShot));
    if (useMotionCompensation) {
      compensatedTarget = targetCenter.minus(fieldVelocity.times(targetLeadSec));
    }

    ShooterCalc.ShotSolution shot =
        shooterCalc.calculateShot(robotPose, compensatedTarget, isPassingShot);

    double yawRateRadPerSec = drive.getYawRateRadPerSec();
    double leadSec =
        SmartDashboard.getNumber(
            getMovingShotTurretRotationLeadKey(isPassingShot),
            getMovingShotTurretRotationLeadDefault(isPassingShot));
    double compScale =
        SmartDashboard.getNumber(
            getMovingShotTurretRotationCompScaleKey(isPassingShot),
            getMovingShotTurretRotationCompScaleDefault(isPassingShot));
    double turretRotationCompDeg = Units.radiansToDegrees(yawRateRadPerSec * leadSec * compScale);

    turret.setturrettoangle(shot.getTurretCommandDegrees() + turretRotationCompDeg);
    shooter.shootersetvelocity(shot.getShooterRps());
    shooter.setshooterhood(shot.getHoodPercent());

    double turretReadyToleranceDeg =
        SmartDashboard.getNumber(
            getMovingShotTurretReadyToleranceKey(isPassingShot),
            getMovingShotTurretReadyToleranceDefault(isPassingShot));
    double shooterReadyToleranceRps =
        SmartDashboard.getNumber(
            getMovingShotShooterReadyToleranceKey(isPassingShot),
            getMovingShotShooterReadyToleranceDefault(isPassingShot));
    double autoShootMaxRobotSpeedMps =
        SmartDashboard.getNumber(
            getMovingShotAutoShootMaxRobotSpeedKey(isPassingShot),
            getMovingShotAutoShootMaxRobotSpeedDefault(isPassingShot));
    boolean turretReady = turret.isAtTarget(turretReadyToleranceDeg);
    boolean shooterReady = shooter.isAtSpeed(shot.getShooterRps(), shooterReadyToleranceRps);
    double robotLinearSpeedMps = drive.getLinearSpeedMetersPerSec();
    boolean robotSlowEnough = robotLinearSpeedMps <= autoShootMaxRobotSpeedMps;
    boolean canAutoFeed = autoShootEnabled && turretReady && shooterReady && robotSlowEnough;
    if (autoShootEnabled) {
      if (canAutoFeed) {
        whirlpool.startwhirlpool();
      } else {
        whirlpool.stopwhirlpool();
      }
    }

    SmartDashboard.putNumber("MovingShot/TargetX", targetCenter.getX());
    SmartDashboard.putNumber("MovingShot/TargetY", targetCenter.getY());
    SmartDashboard.putNumber("MovingShot/CompensatedTargetX", compensatedTarget.getX());
    SmartDashboard.putNumber("MovingShot/CompensatedTargetY", compensatedTarget.getY());
    SmartDashboard.putNumber(
        "MovingShot/TargetLeadLookupDistanceMeters", targetLeadLookupDistanceMeters);
    SmartDashboard.putNumber("MovingShot/TargetLeadSecActive", targetLeadSec);
    SmartDashboard.putNumber("MovingShot/DistanceMeters", shot.getDistanceMeters());
    SmartDashboard.putNumber(
        "MovingShot/EffectiveDistanceMeters", shot.getEffectiveDistanceMeters());
    SmartDashboard.putNumber("MovingShot/TurretCmdDeg", shot.getTurretCommandDegrees());
    SmartDashboard.putNumber("MovingShot/ShooterRps", shot.getShooterRps());
    SmartDashboard.putNumber("MovingShot/HoodPercent", shot.getHoodPercent());
    SmartDashboard.putNumber(
        "MovingShot/LateralOffsetMeters", shooterCalc.getLateralAimOffsetMeters());
    SmartDashboard.putNumber(
        "MovingShot/DistanceOffsetMeters", shooterCalc.getDistanceOffsetMeters());
    SmartDashboard.putNumber("MovingShot/YawRateRadPerSec", yawRateRadPerSec);
    SmartDashboard.putNumber("MovingShot/RobotLinearSpeedMps", robotLinearSpeedMps);
    SmartDashboard.putNumber("MovingShot/RobotFieldVxMps", fieldVelocity.getX());
    SmartDashboard.putNumber("MovingShot/RobotFieldVyMps", fieldVelocity.getY());
    SmartDashboard.putNumber("MovingShot/TurretRotationCompDeg", turretRotationCompDeg);
    SmartDashboard.putBoolean("MovingShot/TurretReady", turretReady);
    SmartDashboard.putBoolean("MovingShot/ShooterReady", shooterReady);
    SmartDashboard.putBoolean("MovingShot/RobotSlowEnough", robotSlowEnough);
    SmartDashboard.putBoolean("MovingShot/IsPassingShot", isPassingShot);
    SmartDashboard.putBoolean("MovingShot/CanAutoFeed", canAutoFeed);
    SmartDashboard.putNumber("MovingShot/DriverLinearScaleActive", getMovingShotDriveLinearScale());
    SmartDashboard.putNumber("MovingShot/DriverOmegaScaleActive", getMovingShotDriveOmegaScale());
  }

  private double getMovingShotDriveLinearScale() {
    if (!flightcontroller.getHID().getRawButton(12)) {
      return 1.0;
    }
    boolean isPassingShot = !isRobotInOwnZone(drive.getPose());
    double maxAllowedSpeedMps =
        SmartDashboard.getNumber(
            getMovingShotAutoShootMaxRobotSpeedKey(isPassingShot),
            getMovingShotAutoShootMaxRobotSpeedDefault(isPassingShot));
    double autoShootScale = maxAllowedSpeedMps / drive.getMaxLinearSpeedMetersPerSec();
    double manualScale =
        SmartDashboard.getNumber(
            getMovingShotDriverMaxLinearScaleKey(isPassingShot),
            getMovingShotDriverMaxLinearScaleDefault(isPassingShot));
    return Math.min(manualScale, autoShootScale);
  }

  private double getMovingShotDriveOmegaScale() {
    if (!flightcontroller.getHID().getRawButton(12)) {
      return 1.0;
    }
    boolean isPassingShot = !isRobotInOwnZone(drive.getPose());
    return SmartDashboard.getNumber(
        getMovingShotDriverMaxOmegaScaleKey(isPassingShot),
        getMovingShotDriverMaxOmegaScaleDefault(isPassingShot));
  }

  private void adjustDistanceOffsetsInches(double deltaInches) {
    adjustDashboardNumber(AutoAimConstants.DISTANCE_OFFSET_INCHES_KEY, deltaInches);
    adjustDashboardNumber(MovingShotConstants.DISTANCE_OFFSET_INCHES_KEY, deltaInches);
    adjustDashboardNumber(MovingPassShotConstants.DISTANCE_OFFSET_INCHES_KEY, deltaInches);
  }

  private void adjustLateralOffsetsInches(double deltaInches) {
    adjustDashboardNumber(AutoAimConstants.LATERAL_OFFSET_INCHES_KEY, deltaInches);
    adjustDashboardNumber(MovingShotConstants.LATERAL_OFFSET_INCHES_KEY, deltaInches);
    adjustDashboardNumber(MovingPassShotConstants.LATERAL_OFFSET_INCHES_KEY, deltaInches);
  }

  private String getMovingShotLateralOffsetKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.LATERAL_OFFSET_INCHES_KEY
        : MovingShotConstants.LATERAL_OFFSET_INCHES_KEY;
  }

  private double getMovingShotLateralOffsetDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.LATERAL_OFFSET_INCHES_DEFAULT
        : MovingShotConstants.LATERAL_OFFSET_INCHES_DEFAULT;
  }

  private String getMovingShotDistanceOffsetKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DISTANCE_OFFSET_INCHES_KEY
        : MovingShotConstants.DISTANCE_OFFSET_INCHES_KEY;
  }

  private double getMovingShotDistanceOffsetDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DISTANCE_OFFSET_INCHES_DEFAULT
        : MovingShotConstants.DISTANCE_OFFSET_INCHES_DEFAULT;
  }

  private String getMovingShotTurretRotationLeadKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_ROTATION_LEAD_SECONDS_KEY
        : MovingShotConstants.TURRET_ROTATION_LEAD_SECONDS_KEY;
  }

  private double getMovingShotTurretRotationLeadDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT
        : MovingShotConstants.TURRET_ROTATION_LEAD_SECONDS_DEFAULT;
  }

  private String getMovingShotTurretRotationCompScaleKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_ROTATION_COMP_SCALE_KEY
        : MovingShotConstants.TURRET_ROTATION_COMP_SCALE_KEY;
  }

  private double getMovingShotTurretRotationCompScaleDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT
        : MovingShotConstants.TURRET_ROTATION_COMP_SCALE_DEFAULT;
  }

  private String getMovingShotTurretReadyToleranceKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_READY_TOLERANCE_DEG_KEY
        : MovingShotConstants.TURRET_READY_TOLERANCE_DEG_KEY;
  }

  private double getMovingShotTurretReadyToleranceDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT
        : MovingShotConstants.TURRET_READY_TOLERANCE_DEG_DEFAULT;
  }

  private String getMovingShotShooterReadyToleranceKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.SHOOTER_READY_TOLERANCE_RPS_KEY
        : MovingShotConstants.SHOOTER_READY_TOLERANCE_RPS_KEY;
  }

  private double getMovingShotShooterReadyToleranceDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT
        : MovingShotConstants.SHOOTER_READY_TOLERANCE_RPS_DEFAULT;
  }

  private String getMovingShotAutoShootMaxRobotSpeedKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY
        : MovingShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_KEY;
  }

  private double getMovingShotAutoShootMaxRobotSpeedDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT
        : MovingShotConstants.AUTO_SHOOT_MAX_ROBOT_SPEED_MPS_DEFAULT;
  }

  private String getMovingShotDriverMaxLinearScaleKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DRIVER_MAX_LINEAR_SCALE_KEY
        : MovingShotConstants.DRIVER_MAX_LINEAR_SCALE_KEY;
  }

  private double getMovingShotDriverMaxLinearScaleDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DRIVER_MAX_LINEAR_SCALE_DEFAULT
        : MovingShotConstants.DRIVER_MAX_LINEAR_SCALE_DEFAULT;
  }

  private String getMovingShotDriverMaxOmegaScaleKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DRIVER_MAX_OMEGA_SCALE_KEY
        : MovingShotConstants.DRIVER_MAX_OMEGA_SCALE_KEY;
  }

  private double getMovingShotDriverMaxOmegaScaleDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.DRIVER_MAX_OMEGA_SCALE_DEFAULT
        : MovingShotConstants.DRIVER_MAX_OMEGA_SCALE_DEFAULT;
  }

  private String getMovingShotUseMotionCompensationKey(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.USE_MOTION_COMPENSATION_KEY
        : MovingShotConstants.USE_MOTION_COMPENSATION_KEY;
  }

  private boolean getMovingShotUseMotionCompensationDefault(boolean isPassingShot) {
    return isPassingShot
        ? MovingPassShotConstants.USE_MOTION_COMPENSATION_DEFAULT
        : MovingShotConstants.USE_MOTION_COMPENSATION_DEFAULT;
  }

  private NavigableMap<Double, Double> getMovingShotTargetLeadTable(boolean isPassingShot) {
    return isPassingShot ? buildMovingPassTargetLeadTable() : buildMovingShotTargetLeadTable();
  }

  private void adjustDashboardNumber(String key, double delta) {
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, 0.0) + delta);
  }

  private static double interpolateLookup(double x, NavigableMap<Double, Double> table) {
    if (table.isEmpty()) {
      return 0.0;
    }
    var lower = table.floorEntry(x);
    var upper = table.ceilingEntry(x);
    if (lower == null) {
      return upper.getValue();
    }
    if (upper == null) {
      return lower.getValue();
    }
    if (Math.abs(upper.getKey() - lower.getKey()) < 1e-9) {
      return lower.getValue();
    }
    double t = (x - lower.getKey()) / (upper.getKey() - lower.getKey());
    return lower.getValue() + t * (upper.getValue() - lower.getValue());
  }

  private Translation2d getCurrentAllianceAimTarget(Pose2d robotPose) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    double zoneSplitYMeters =
        SmartDashboard.getNumber(
            AutoAimConstants.ZONE_SPLIT_Y_METERS_KEY, AutoAimConstants.ZONE_SPLIT_Y_METERS_DEFAULT);
    double ownZoneMaxXMeters =
        SmartDashboard.getNumber(
            AutoAimConstants.OWN_ZONE_MAX_X_METERS_KEY,
            AutoAimConstants.OWN_ZONE_MAX_X_METERS_DEFAULT);
    Translation2d bluePassBottomTarget =
        new Translation2d(
            SmartDashboard.getNumber(
                AutoAimConstants.BLUE_PASS_BOTTOM_X_KEY,
                AutoAimConstants.BLUE_PASS_TARGET_BOTTOM_DEFAULT_FIELD_METERS.getX()),
            SmartDashboard.getNumber(
                AutoAimConstants.BLUE_PASS_BOTTOM_Y_KEY,
                AutoAimConstants.BLUE_PASS_TARGET_BOTTOM_DEFAULT_FIELD_METERS.getY()));
    Translation2d bluePassTopTarget =
        new Translation2d(
            SmartDashboard.getNumber(
                AutoAimConstants.BLUE_PASS_TOP_X_KEY,
                AutoAimConstants.BLUE_PASS_TARGET_TOP_DEFAULT_FIELD_METERS.getX()),
            SmartDashboard.getNumber(
                AutoAimConstants.BLUE_PASS_TOP_Y_KEY,
                AutoAimConstants.BLUE_PASS_TARGET_TOP_DEFAULT_FIELD_METERS.getY()));
    Translation2d redPassBottomTarget =
        new Translation2d(
            AutoAimConstants.FIELD_LENGTH_METERS - bluePassBottomTarget.getX(),
            bluePassBottomTarget.getY());
    Translation2d redPassTopTarget =
        new Translation2d(
            AutoAimConstants.FIELD_LENGTH_METERS - bluePassTopTarget.getX(),
            bluePassTopTarget.getY());

    boolean inOwnZone = isRobotInOwnZone(robotPose);
    if (inOwnZone) {
      return alliance == DriverStation.Alliance.Red
          ? kRedGoalCenterFieldMeters
          : kBlueGoalCenterFieldMeters;
    }

    boolean useBottomPassTarget = robotPose.getY() < zoneSplitYMeters;
    if (alliance == DriverStation.Alliance.Red) {
      return useBottomPassTarget ? redPassBottomTarget : redPassTopTarget;
    }
    return useBottomPassTarget ? bluePassBottomTarget : bluePassTopTarget;
  }

  private boolean isRobotInOwnZone(Pose2d robotPose) {
    DriverStation.Alliance alliance =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    double ownZoneMaxXMeters =
        SmartDashboard.getNumber(
            AutoAimConstants.OWN_ZONE_MAX_X_METERS_KEY,
            AutoAimConstants.OWN_ZONE_MAX_X_METERS_DEFAULT);
    double xFromOwnWallMeters =
        alliance == DriverStation.Alliance.Blue
            ? robotPose.getX()
            : AutoAimConstants.FIELD_LENGTH_METERS - robotPose.getX();
    return xFromOwnWallMeters <= ownZoneMaxXMeters;
  }

  private boolean isRobotInPassNoShootZone(Pose2d robotPose) {
    double passNoShootCenterY =
        SmartDashboard.getNumber(
            AutoAimConstants.PASS_NO_SHOOT_CENTER_Y_KEY,
            AutoAimConstants.PASS_NO_SHOOT_CENTER_Y_DEFAULT);
    double passNoShootWidthMeters =
        SmartDashboard.getNumber(
            AutoAimConstants.PASS_NO_SHOOT_WIDTH_METERS_KEY,
            AutoAimConstants.PASS_NO_SHOOT_WIDTH_METERS_DEFAULT);
    double halfWidthMeters = Math.max(0.0, passNoShootWidthMeters) / 2.0;
    return Math.abs(robotPose.getY() - passNoShootCenterY) <= halfWidthMeters;
  }

  private static NavigableMap<Double, Double> buildShooterRpsTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.565, 28.0);
    table.put(2.014, 29.5);
    table.put(2.455, 31.0);
    table.put(2.971, 33.0);
    table.put(3.581, 35.0);
    table.put(4.1, 37.0);
    table.put(4.5, 38.5);
    table.put(4.96, 39.5);
    table.put(5.391, 40.0); // was 43
    table.put(5.500, 42.0);
    table.put(6.287, 50.0);
    table.put(12.5, 80.0);
    return table;
  }

  private static NavigableMap<Double, Double> buildHoodPercentTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.565, 0.0);
    table.put(2.014, 0.05);
    table.put(2.455, 0.09);
    table.put(2.971, 0.12);
    table.put(3.581, 0.16);
    table.put(4.1, 0.19);
    table.put(4.5, 0.21);
    table.put(4.96, 0.23);
    table.put(5.391, 0.27);
    table.put(5.500, .28);
    table.put(6.287, 0.70);
    table.put(12.5, 0.75);
    return table;
  }

  private static NavigableMap<Double, Double> buildShooterPassing() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.565, 26.0);
    table.put(2.014, 28.0);
    table.put(2.455, 29.0);
    table.put(2.971, 31.0);
    table.put(3.581, 35.0);
    table.put(4.1, 37.0);
    table.put(4.5, 39.0);
    table.put(4.96, 40.0);
    table.put(5.391, 40.5); // was 43
    table.put(5.500, 40.0);
    table.put(6.287, 45.0);
    table.put(12.5, 65.0);
    return table;
  }

  private static NavigableMap<Double, Double> buildShooterPassinghood() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(1.565, 0.0 + hoodpassingaddition);
    table.put(2.014, 0.05 + hoodpassingaddition);
    table.put(2.455, 0.09 + hoodpassingaddition);
    table.put(2.971, 0.12 + hoodpassingaddition);
    table.put(3.581, 0.16 + hoodpassingaddition);
    table.put(3.581, 0.16 + hoodpassingaddition);
    table.put(3.581, 0.16 + hoodpassingaddition);
    table.put(4.1, 0.19 + hoodpassingaddition);
    table.put(4.5, 0.21 + hoodpassingaddition);
    table.put(4.96, 0.23 + hoodpassingaddition);
    table.put(5.391, 0.27 + hoodpassingaddition);
    table.put(5.500, .28 + hoodpassingaddition);
    table.put(6.287, 0.70 + hoodpassingaddition);
    table.put(12.5, 0.75 + hoodpassingaddition);
    return table;
  }

  private static NavigableMap<Double, Double> buildMovingShotTargetLeadTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(2.0, .6);
    table.put(4.0, .8);
    table.put(5.4, 1.1);
    return table;
  }

  private static NavigableMap<Double, Double> buildMovingPassTargetLeadTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(2.5, 1.25);
    table.put(5.0, 1.5);
    table.put(7.0, 1.75);
    return table;
  }
}

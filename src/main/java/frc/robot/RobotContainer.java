// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.Direction;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.Alerts;
import frc.robot.util.AutoShotCalculator;
import frc.robot.util.Elastic;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PathPlanOnTheFly;
import frc.robot.util.Rumble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
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
  private final Turret turret;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Hood hood;
  private final Spindexer spindexer;
  private final Intake intake;
  private final Leds leds;

  // vision systemns
  private final Vision vision;
  private final VisionIOQuestNav questNavIO;
  private final VisionIOLimelight limelightIO;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController[] bothControllers = {operatorController, driveController};
  // private final CommandJoystick operatorPanel = new CommandJoystick(1);
  private final Field2d m_field = new Field2d();

  private final AutoShotCalculator shotCalculator;
  private AutoShotCalculator.ShotSolution latestSolution = AutoShotCalculator.ShotSolution.none();
  private boolean isAutoAiming = false;
  private boolean isStaticAiming = false;
  private final Trigger isUnableToShoot = new Trigger(() -> !latestSolution.isSolutionFound());
  private static final LoggedTunableNumber hoodOffset =
      new LoggedTunableNumber("AutoShot/hoodOffset", 0.122);

  // private final Trigger isUnableToShoot = new
  // Trigger(shotCalculator::isUnableToShoot);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    questNavIO = new VisionIOQuestNav("questnav-fl");

    Logger.recordOutput(
        "AutoShot/leftPassTargetBlue",
        new Pose3d(FieldConstants.PassTarget.leftPassTargetBlue, Rotation3d.kZero));
    Logger.recordOutput(
        "AutoShot/rightPassTargetBlue",
        new Pose3d(FieldConstants.PassTarget.rightPassTargetBlue, Rotation3d.kZero));
    Logger.recordOutput(
        "AutoShot/leftPassTargetRed",
        new Pose3d(FieldConstants.PassTarget.leftPassTargetRed, Rotation3d.kZero));
    Logger.recordOutput(
        "AutoShot/rightPassTargetRed",
        new Pose3d(FieldConstants.PassTarget.rightPassTargetRed, Rotation3d.kZero));

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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                questNavIO::setInitialPose);

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
                new ModuleIOSim(TunerConstants.BackRight),
                questNavIO::setInitialPose);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                questNavIO::setInitialPose);
        break;
    }

    // vision instantiation
    limelightIO = new VisionIOLimelight(VisionConstants.limelightName, drive::getRotation);
    vision = new Vision(drive::addVisionMeasurement, questNavIO, limelightIO);

    // subsystems
    turret = new Turret(new TurretIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX());
    kicker = new Kicker(new KickerIOTalonFX());
    hood = new Hood(new HoodIOTalonFX());
    spindexer = new Spindexer(new SpindexerIOTalonFX());
    intake = new Intake(new IntakeIOTalonFX());
    leds = new Leds();

    // named command registration
    NamedCommands.registerCommand(
        "ExtendIntake", intake.ExtendIntake().deadlineFor(leds.WhiteColorFlow()));
    NamedCommands.registerCommand("RunIntake", intake.RunIntake(() -> false));
    NamedCommands.registerCommand("StartIntake", intake.StartIntake());
    NamedCommands.registerCommand("StopIntake", intake.StopIntake());
    NamedCommands.registerCommand("AutoAim", AutoAim(this::getHubTarget));
    NamedCommands.registerCommand("FeedBallsToShooter", FeedBallsToShooter());
    NamedCommands.registerCommand("AimAndShootBalls4Sec", AimAndShootBalls(4));
    NamedCommands.registerCommand("AimAndShootBalls4Ever", AimAndShootBalls());
    NamedCommands.registerCommand("PushBallsIntoSpindexer", intake.PushBallsIntoSpindexer());

    shotCalculator = new AutoShotCalculator(turret);

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

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

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
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(Commands.runOnce(() -> drive.resetGyroOffset(), drive).ignoringDisable(true));
    driveController.leftTrigger().whileTrue(intake.RunIntake(driveController.x()));
    driveController.leftBumper().whileTrue(intake.RunOuttake(driveController.x()));

    operatorController.a().and(isUnableToShoot).whileTrue(Rumble.rumble(bothControllers, 1.0));
    operatorController.b().and(isUnableToShoot).whileTrue(Rumble.rumble(bothControllers, 1.0));

    operatorController.x().whileTrue(AimToDrop().alongWith(intake.RunOuttake(() -> false)));

    operatorController.leftBumper().onTrue(intake.PushBallsIntoSpindexer());

    operatorController
        .povLeft()
        .and(this::isAiming)
        .whileTrue(turret.AddTurretOffset(Direction.CCW, this::isStaticAiming));
    operatorController
        .povRight()
        .and(this::isAiming)
        .whileTrue(turret.AddTurretOffset(Direction.CW, this::isStaticAiming));

    operatorController
        .a()
        .and(() -> !isStaticAiming())
        .whileTrue(
            AutoAim(this::getHubTarget)
                .deadlineFor(leds.TrueFalseColorFlow(isUnableToShoot.negate()))
                .onlyWhile(() -> !isStaticAiming()));

    operatorController
        .b()
        .and(() -> !isStaticAiming())
        .whileTrue(
            AutoAimToPass(() -> getPassTarget(drive.getPose()))
                .onlyWhile(() -> !isStaticAiming()));
    operatorController.rightTrigger().whileTrue(FeedBallsToShooter());
    operatorController
        .rightBumper()
        .onTrue(intake.ExtendIntake().deadlineFor(leds.WhiteColorFlow()));

    operatorController
        .button(7)
        .toggleOnTrue(
            StaticAim(this::getHubTarget, this::getStaticShootPose)
                .deadlineFor(leds.OrangeStrobe()));

    operatorController.button(7).onTrue(Rumble.rumblePattern(bothControllers, 1, 0.1, 0.1, 5));

    driveController.x().whileTrue(PathPlanOnTheFly.NavigateTrench(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command AimAndShootBalls() {
    return Commands.parallel(
        AutoAim(this::getHubTarget),
        Commands.sequence(
            Commands.waitUntil(() -> latestSolution.isSolutionFound()),
            Commands.waitUntil(shooter::isAtGoalSpeed),
            Commands.waitUntil(turret::isAtGoalPosition),
            FeedBallsToShooter()));
  }

  public Command AimAndShootBalls(double timeout) {
    return Commands.parallel(
            AutoAim(this::getHubTarget),
            Commands.sequence(
                Commands.waitUntil(() -> latestSolution.isSolutionFound()),
                Commands.waitUntil(shooter::isAtGoalSpeed),
                Commands.waitUntil(turret::isAtGoalPosition),
                FeedBallsToShooter()))
        .withTimeout(timeout);
  }

  public Command FeedBallsToShooter() {
    return Commands.parallel(
        kicker.RunKicker(shooter::isAtGoalSpeed),
        Commands.sequence(Commands.waitUntil(kicker::isAtGoalSpeed), spindexer.RunSpindexer()));
  }

  public Command AutoAim(Supplier<Translation3d> target) {
    return Commands.sequence(
            turret.ResetTurretOffset(),
            Commands.runEnd(
                () -> {
                  isAutoAiming = true;
                  autoAimShooter(target.get());
                },
                () -> {
                  isAutoAiming = false;
                  stopAutoAim();
                },
                turret))
        .withName("AutoAim")
        .finallyDo(
            () -> {
              isAutoAiming = false;
              stopAutoAim();
            });
  }

  public Command AutoAimToPass(Supplier<Translation3d> target) {
    return Commands.sequence(
            turret.ResetTurretOffset(),
            Commands.runEnd(
                () -> {
                  isAutoAiming = true;
                  autoAimShooterToPass(target.get());
                },
                () -> {
                  isAutoAiming = false;
                  stopAutoAim();
                },
                turret))
        .withName("AutoAimToPass")
        .finallyDo(
            () -> {
              isAutoAiming = false;
              stopAutoAim();
            });
  }

  public Command AimToDrop() {
    return Commands.sequence(
            turret.ResetTurretOffset(),
            Commands.runEnd(
                () -> {
                  aimToDropBalls();
                },
                () -> {
                  stopAutoAim();
                },
                turret,
                hood,
                shooter))
        .withName("AimToDrop")
        .finallyDo(
            () -> {
              isAutoAiming = false;
              stopAutoAim();
            });
  }

  public Command StaticAim(Supplier<Translation3d> target, Supplier<Pose2d> staticPose) {
    return Commands.sequence(
            turret.ResetTurretOffset(),
            Commands.runEnd(
                () -> {
                  isStaticAiming = true;
                  staticAimShooter(target.get(), staticPose.get());
                  Alerts.AutoShot.staticAimAlert.set(true);
                },
                () -> {
                  isStaticAiming = false;
                  stopAutoAim();
                },
                turret))
        .withName("StaticAim");
  }

  public Command ShootBalls() {
    return Commands.parallel(
        shooter.RunShooter(),
        Commands.sequence(
            Commands.waitUntil(shooter::isAtGoalSpeed),
            Commands.parallel(kicker.RunKicker(shooter::isAtGoalSpeed), spindexer.RunSpindexer())));
  }

  public Command StopShooting() {
    return Commands.parallel(shooter.StopShooter(), kicker.StopKicker(), spindexer.StopSpindexer());
  }

  public void autoAimShooter(Translation3d target) {
    latestSolution =
        shotCalculator.calculate(
            drive.getPose(), drive.getChassisSpeeds(), target, hood.getCurrentHoodRotation());

    if (latestSolution.isSolutionFound()) {
      Elastic.clearAllNotifications();
      Alerts.AutoShot.resetAutoShotAlerts();
      turret.setGoalPositionRad(latestSolution.turretAngleRad());
      double hoodGoalPosition =
          (Math.PI / 2) - latestSolution.hoodAngleRad() - hoodOffset.getAsDouble();
      hood.setGoalPosition(hoodGoalPosition);
      shooter.setShooterGoalSpeedRadPerSec(latestSolution.flywheelVelocityRadPerSec());
    } else {
      switch (latestSolution.constrainingFactor()) {
        case TURRET_RANGE:
          Alerts.AutoShot.turretCannotReachAlert.set(true);
        case LOCATION:
          Alerts.AutoShot.outOfBoundsAlert.set(true);
        case SHOOTER_RANGE:
          Alerts.AutoShot.shooterRangeAlert.set(true);
      }
    }
  }

  public void autoAimShooterToPass(Translation3d target) {
    latestSolution =
        shotCalculator.calculate(
            drive.getPose(), drive.getChassisSpeeds(), target, hood.getCurrentHoodRotation());

    if (latestSolution.isSolutionFound()) {
      Elastic.clearAllNotifications();
      Alerts.AutoShot.resetAutoShotAlerts();
      turret.setGoalPositionRad(latestSolution.turretAngleRad());
      double hoodGoalPosition = (Math.PI / 2) - 0.48 - hoodOffset.getAsDouble();
      hood.setGoalPosition(hoodGoalPosition);
      // shooter.setShooterGoalSpeedRadPerSec(latestSolution.flywheelVelocityRadPerSec());
      shooter.setShooterGoalSpeedRadPerSec(500);
    } else {
      switch (latestSolution.constrainingFactor()) {
        case TURRET_RANGE:
          Alerts.AutoShot.turretCannotReachAlert.set(true);
        case LOCATION:
          Alerts.AutoShot.outOfBoundsAlert.set(true);
        case SHOOTER_RANGE:
          Alerts.AutoShot.shooterRangeAlert.set(true);
      }
    }
  }

  public void aimToDropBalls() {
    turret.setGoalPositionRad(0);
    double hoodGoalPosition = (Math.PI / 2) - 1.15 - hoodOffset.getAsDouble();
    hood.setGoalPosition(hoodGoalPosition);
    shooter.setShooterGoalSpeedRadPerSec(190);
  }

  public void staticAimShooter(Translation3d target, Pose2d staticPose) {
    ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0, 0, 0);
    latestSolution =
        shotCalculator.calculate(staticPose, zeroSpeeds, target, hood.getCurrentHoodRotation());

    if (latestSolution.isSolutionFound()) {
      Elastic.clearAllNotifications();
      Alerts.AutoShot.resetAutoShotAlerts();
      turret.setGoalPositionRad(latestSolution.turretAngleRad());
      double hoodGoalPosition =
          (Math.PI / 2) - latestSolution.hoodAngleRad() - hoodOffset.getAsDouble();
      hood.setGoalPosition(hoodGoalPosition);
      shooter.setShooterGoalSpeedRadPerSec(latestSolution.flywheelVelocityRadPerSec());
    } else {
      switch (latestSolution.constrainingFactor()) {
        case TURRET_RANGE:
          Alerts.AutoShot.turretCannotReachAlert.set(true);
        case LOCATION:
          Alerts.AutoShot.outOfBoundsAlert.set(true);
        case SHOOTER_RANGE:
          Alerts.AutoShot.shooterRangeAlert.set(true);
      }
    }
  }

  public void stopAutoAim() {
    Alerts.AutoShot.resetAutoShotAlerts();
    shooter.setShooterGoalSpeedRadPerSec(0);
    hood.setGoalPosition(0);
  }

  private Translation3d getHubTarget() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    return isRed ? FieldConstants.Hub.oppTopCenterPoint : FieldConstants.Hub.topCenterPoint;
  }

  private Pose2d getStaticShootPose() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    if (isRed) {
      return FieldConstants.StaticShoot.redShoot;
    } else {
      return FieldConstants.StaticShoot.blueShoot;
    }
  }

  private Translation3d getClosestPassTarget(Pose2d currentPose, Translation3d[] passTargets) {
    Translation3d closestTarget = passTargets[0];
    double closestDistance =
        currentPose.getTranslation().getDistance(closestTarget.toTranslation2d());
    for (int i = 0; i < passTargets.length; i++) {
      Translation3d target = passTargets[i];
      double distance = currentPose.getTranslation().getDistance(target.toTranslation2d());
      if (distance < closestDistance) {
        closestTarget = target;
        closestDistance = distance;
      }
    }
    return closestTarget;
  }

  private Translation3d getPassTarget(Pose2d currentPose) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Logger.recordOutput("AutoShot/PassTargets", FieldConstants.PassTarget.bluePassTargets);

    Translation3d target =
        isRed
            ? getClosestPassTarget(currentPose, FieldConstants.PassTarget.redPassTargets)
            : getClosestPassTarget(currentPose, FieldConstants.PassTarget.bluePassTargets);

    Logger.recordOutput("AutoShot/CurrentPose", currentPose);
    Logger.recordOutput("AutoShot/Target", target);
    return target;
  }

  @AutoLogOutput
  private boolean isStaticAiming() {
    return isStaticAiming;
  }

  private boolean isAiming() {
    return isStaticAiming || isAutoAiming;
  }
}

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Pneumatics.Pneumatics;
import frc.robot.subsystems.Pneumatics.PneumaticsIO;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.spark.ModuleIOSpark;
import frc.robot.subsystems.drive.spark.ModuleIOSparkSim;
import frc.robot.subsystems.drive.spark.SparkMaxModuleConstants;
import frc.robot.subsystems.drive.spark.SparkOdometryThread;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
//   private final LEDS led;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Pneumatics doubleSolenoid;
  // Simulation
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber xOverride;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(SparkMaxModuleConstants.frontLeft),
                new ModuleIOSpark(SparkMaxModuleConstants.frontRight),
                new ModuleIOSpark(SparkMaxModuleConstants.rearLeft),
                new ModuleIOSpark(SparkMaxModuleConstants.rearRight),
                SparkOdometryThread.getInstance());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight", () -> drive.getPose().getRotation()));
                doubleSolenoid = new Pneumatics(new PneumaticsIO() {});
                // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIONeo("Elevator", ElevatorConstants.EXAMPLE_CONFIG),
                new ElevatorGains(
                    ElevatorConstants.EXAMPLE_GAINS.kP(),
                    ElevatorConstants.EXAMPLE_GAINS.kI(),
                    ElevatorConstants.EXAMPLE_GAINS.kD(),
                    ElevatorConstants.EXAMPLE_GAINS.kS(),
                    ElevatorConstants.EXAMPLE_GAINS.kG(),
                    ElevatorConstants.EXAMPLE_GAINS.kV(),
                    ElevatorConstants.EXAMPLE_GAINS.kA(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxVelo(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxAccel(),
                    ElevatorConstants.EXAMPLE_GAINS.kMinPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kTolerance()));

        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSparkSim(driveSimulation.getModules()[0]),
                new ModuleIOSparkSim(driveSimulation.getModules()[1]),
                new ModuleIOSparkSim(driveSimulation.getModules()[2]),
                new ModuleIOSparkSim(driveSimulation.getModules()[3]),
                null);
        doubleSolenoid = new Pneumatics(new PneumaticsIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIOLimelight("", ()->new Rotation2d()));
        // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),
                new ElevatorGains(
                    ElevatorConstants.EXAMPLE_GAINS.kP(),
                    ElevatorConstants.EXAMPLE_GAINS.kI(),
                    ElevatorConstants.EXAMPLE_GAINS.kD(),
                    ElevatorConstants.EXAMPLE_GAINS.kS(),
                    ElevatorConstants.EXAMPLE_GAINS.kG(),
                    ElevatorConstants.EXAMPLE_GAINS.kV(),
                    ElevatorConstants.EXAMPLE_GAINS.kA(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxVelo(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxAccel(),
                    ElevatorConstants.EXAMPLE_GAINS.kMinPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kTolerance()));
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
                null);
        doubleSolenoid = new Pneumatics(new PneumaticsIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),
                new ElevatorGains(
                    ElevatorConstants.EXAMPLE_GAINS.kP(),
                    ElevatorConstants.EXAMPLE_GAINS.kI(),
                    ElevatorConstants.EXAMPLE_GAINS.kD(),
                    ElevatorConstants.EXAMPLE_GAINS.kS(),
                    ElevatorConstants.EXAMPLE_GAINS.kG(),
                    ElevatorConstants.EXAMPLE_GAINS.kV(),
                    ElevatorConstants.EXAMPLE_GAINS.kA(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxVelo(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxAccel(),
                    ElevatorConstants.EXAMPLE_GAINS.kMinPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kMaxPosition(), 
                    ElevatorConstants.EXAMPLE_GAINS.kTolerance()));
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    xOverride = new LoggedNetworkNumber("/PPOverrides", 0.0);

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
    // led.runLEDS();
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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.setPose(
                    new Pose2d(
                        drive.getPose().getTranslation(),
                        DriverStation.getAlliance().isPresent()
                            ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                                ? new Rotation2d(Math.PI)
                                : new Rotation2d())
                            : new Rotation2d())); // zero gyro
    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    driverController.y().whileTrue(drive.generatePath(new Pose2d(3.589,5.334, Rotation2d.fromDegrees(-128.721))));
    driverController.povLeft().onTrue(Commands.run(()->doubleSolenoid.setMode(Value.kForward)));
    driverController.povRight().onTrue(Commands.run(()->doubleSolenoid.setMode(Value.kOff)));
    // driverController.a().onTrue(Commands.run(() -> elevator.periodic(), elevator));

    AdvancedPPHolonomicDriveController.setYSetpointIncrement(xOverride::get);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(drive.getPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}

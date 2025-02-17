package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ManipulatorStart;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIO;
import frc.robot.subsystems.Manipulator.ManipulatorIOSim;
import frc.robot.subsystems.Manipulator.ManipulatorConstants.ManipulatorHardwareConfig;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIO;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
import frc.robot.subsystems.beam_break.BeamBreakConstants.BeamBreakConfig;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSim;
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
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
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
//   private final Elevator elevator;
//   private final LEDS led;
  @SuppressWarnings("unused")
  private final Vision vision;
  // Simulation
  private SwerveDriveSimulation driveSimulation = null;
  private final Manipulator shooter;
  private final Indexer indexer;
  //beam breaks
  private final BeamBreak beamBreakBack;
  private final BeamBreak beamBreakMid;
  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
   //triggers
  private final Trigger yIsPressed = new Trigger(driverController.y());
  private final Trigger povDownisPressed = new Trigger(driverController.povDown());
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber xOverride;
  //command definitions
  private Command ManipulatorShoot;
  private Command ManipulatorStop;
  private Command ManipulatorClear;
  private Command indexerStart;
  private Command indexerStop;
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
                shooter = new Manipulator(new ManipulatorIO() {}, ManipulatorConstants.REAL_GAINS);
                indexer = new Indexer(new IndexerIO() {}, IndexerConstants.SIM_GAINS);
                beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
                beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
                // led = new LEDS(60);
                // elevator =
                //     new Elevator(
                //         new ElevatorIONeo("Elevator", ElevatorConstants.EXAMPLE_CONFIG),
                //         new ElevatorGains(
                //             ElevatorConstants.EXAMPLE_GAINS.kP(),
                //             ElevatorConstants.EXAMPLE_GAINS.kI(),
                //             ElevatorConstants.EXAMPLE_GAINS.kD(),
                //             ElevatorConstants.EXAMPLE_GAINS.kS(),
                //             ElevatorConstants.EXAMPLE_GAINS.kV(),
                //             ElevatorConstants.EXAMPLE_GAINS.kA()));
        
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
                shooter = new Manipulator(new ManipulatorIOSim("shooter", ManipulatorConstants.EXAMPLE_CONFIG), ManipulatorConstants.SIM_GAINS);
                indexer = new Indexer(new IndexerIOSim("indexerSim",IndexerConstants.EXAMPLE_CONFIG) {}, IndexerConstants.SIM_GAINS);
                beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
                beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
                vision = new Vision(drive::addVisionMeasurement, new VisionIOLimelight("", ()->new Rotation2d()));
                // led = new LEDS(60);
                // elevator =
                //     new Elevator(
                //         new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),
                //         new ElevatorGains(
                //             ElevatorConstants.EXAMPLE_GAINS.kP(),
                //             ElevatorConstants.EXAMPLE_GAINS.kI(),
                //             ElevatorConstants.EXAMPLE_GAINS.kD(),
                //             ElevatorConstants.EXAMPLE_GAINS.kS(),
                //             ElevatorConstants.EXAMPLE_GAINS.kV(),
                //             ElevatorConstants.EXAMPLE_GAINS.kA()));
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
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                shooter = new Manipulator(new ManipulatorIOSim("shooter", ManipulatorConstants.EXAMPLE_CONFIG) {}, ManipulatorConstants.SIM_GAINS);
                indexer = new Indexer(new IndexerIOSim("indexerSim",IndexerConstants.EXAMPLE_CONFIG) {}, IndexerConstants.SIM_GAINS);
                beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
                beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
                // led = new LEDS(60);
                // elevator =
                //     new Elevator(
                //         new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),
                //         new ElevatorGains(
                //             ElevatorConstants.EXAMPLE_GAINS.kP(),
                //             ElevatorConstants.EXAMPLE_GAINS.kI(),
                //             ElevatorConstants.EXAMPLE_GAINS.kD(),
                //             ElevatorConstants.EXAMPLE_GAINS.kS(),
                //             ElevatorConstants.EXAMPLE_GAINS.kV(),
                //             ElevatorConstants.EXAMPLE_GAINS.kA()));
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
            //Commands
            ManipulatorShoot = Commands.run(()->shooter.setVelocity(10)).withTimeout(15);
            ManipulatorStop = Commands.run(()->shooter.setVelocity(0));
            ManipulatorClear = Commands.run(()->shooter.setVelocity(-10)).withTimeout(3).andThen(ManipulatorStop); //runs motor backwards to get rid of coral from manipulator
            indexerStart = Commands.run(()->indexer.setVelocity(10)).withTimeout(5);
            indexerStop = Commands.run(()->indexer.setVelocity(0));
        
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
    yIsPressed.whileFalse(ManipulatorStop).whileTrue(ManipulatorShoot);
    povDownisPressed.whileFalse(indexerStop).whileTrue(indexerStart);
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
    // driverController.y().whileTrue(drive.generatePath(new Pose2d(3.589,5.334, Rotation2d.fromDegrees(-128.721))));
    driverController.povUp().whileTrue(drive.generatePath(new Pose2d(3.483,7.142, Rotation2d.fromDegrees(108.814))));
    driverController.povLeft().whileTrue(Commands.run(()->shooter.setVelocity(100))).whileFalse(ManipulatorShoot);
    // driverController.y().and(shootCommandTrigger.negate()).whileTrue(ManipulatorShoot);
    driverController.povRight().whileTrue(new frc.robot.commands.CoralAlignment(shooter,beamBreakMid,beamBreakBack));
    // driverController.a().onTrue(Commands.run(() -> elevator.periodic(), elevator));
    driverController.leftStick().whileTrue(ManipulatorClear);
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

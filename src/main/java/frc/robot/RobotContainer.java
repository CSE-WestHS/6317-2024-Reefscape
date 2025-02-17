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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FunnelUp;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeArmConstants;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIO;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
// import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Funnel.FunnelIOReplay;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIOSim;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Funnel.FunnelIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIOSim;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Funnel.FunnelIO;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIO;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.spark.ModuleIOSpark;
import frc.robot.subsystems.drive.spark.ModuleIOSparkSim;
import frc.robot.subsystems.drive.spark.SparkMaxModuleConstants;
import frc.robot.subsystems.drive.spark.SparkOdometryThread;
// import frc.robot.subsystems.drive.talon.ModuleIOTalonFX;
// import frc.robot.subsystems.drive.talon.PhoenixOdometryThread;
// import frc.robot.subsystems.drive.talon.TalonFXModuleConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.ButtonBoardButtons;
import frc.robot.util.UtilitiesFieldSectioning;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.commands.GoToPositionElevator;
import frc.robot.commands.AlgaeArmCommands.AlgaeArmPositionCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
//   private final Elevator elevator;
//   private final LEDS led;
  
  // Simulation
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  
  //buttonboard
  private static final CommandJoystick buttonboardController = new CommandJoystick(1);

  //triggers
  private final Trigger yIsPressed = new Trigger(driverController.y());
  private final Trigger povDownisPressed = new Trigger(driverController.povDown());
  private Trigger elevatorButtonTrigger = new Trigger(driverController.rightStick());
  private final Trigger leftXTrigger = new Trigger(()->(Math.abs(driverController.getLeftX()))>DriveCommands.DEADBAND);
  private final Trigger leftYTrigger = new Trigger(()->(Math.abs(driverController.getLeftY()))>DriveCommands.DEADBAND);
  private final Trigger rightXTrigger = new Trigger(()->(Math.abs(driverController.getRightX()))>DriveCommands.DEADBAND);
  private final Trigger allTrigger = new Trigger(()->leftXTrigger.getAsBoolean() || leftYTrigger.getAsBoolean() || rightXTrigger.getAsBoolean());
  //Subsystem Definitions
  private final Drive drive;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Manipulator shooter;
  private final Indexer indexer;
  private final BeamBreak beamBreakBack;
  private final BeamBreak beamBreakMid;
  private final Funnel funnel;
  private final Elevator elevator;
  private final AlgaeArm algaeArm;
  public static final LEDS led = new LEDS(10); //TODO: Change length based on new robot leds
  //commands
  private Command ManipulatorShoot; 
  private Command ManipulatorStop;
  private Command ManipulatorClear;
  private Command indexerStart;
  private Command indexerStop;
  private Command AlgaeArmPositionSet;
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
        shooter = new Manipulator(new ManipulatorIO() {}, ManipulatorConstants.REAL_GAINS);
        indexer = new Indexer(new IndexerIO() {}, IndexerConstants.SIM_GAINS);
        beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
        beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
        funnel = new Funnel(new FunnelIO() {}, FunnelConstants.REAL_GAINS);
        algaeArm = new AlgaeArm(new AlgaeArmIO() {}, AlgaeArmConstants.EXAMPLE_GAINS);
        // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIONeo("Elevator", ElevatorConstants.EXAMPLE_CONFIG),
               ElevatorConstants.EXAMPLE_GAINS);

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIOLimelight("", ()->new Rotation2d()));
        shooter = new Manipulator(new ManipulatorIOSim("shooter", ManipulatorConstants.EXAMPLE_CONFIG), ManipulatorConstants.SIM_GAINS);
        indexer = new Indexer(new IndexerIOSim("indexerSim",IndexerConstants.EXAMPLE_CONFIG) {}, IndexerConstants.SIM_GAINS);
        beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
        beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
        funnel = new Funnel(new FunnelIOSim("funnelSim", FunnelConstants.EXAMPLE_CONFIG), FunnelConstants.SIM_GAINS);
        algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.EXAMPLE_CONFIG), AlgaeArmConstants.EXAMPLE_GAINS);
        // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),ElevatorConstants.EXAMPLE_GAINS);
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
        funnel = new Funnel(new FunnelIOReplay("funnelReplay"), FunnelConstants.SIM_GAINS);
        algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.EXAMPLE_CONFIG), AlgaeArmConstants.EXAMPLE_GAINS);
        // led = new LEDS(60);
        elevator =
            new Elevator(
                new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),ElevatorConstants.EXAMPLE_GAINS);
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
    // command definitions
    ManipulatorShoot = Commands.run(()->shooter.setVelocity(10)).withTimeout(15);
    ManipulatorStop = Commands.run(()->shooter.setVelocity(0));
    ManipulatorClear = Commands.run(()->shooter.setVelocity(-10)).withTimeout(3).andThen(ManipulatorStop); //runs motor backwards to get rid of coral from manipulator
    indexerStart = Commands.run(()->indexer.setVelocity(10)).withTimeout(5);
    indexerStop = Commands.run(()->indexer.setVelocity(0));
    AlgaeArmPositionSet = Commands.run(()->algaeArm.setPosition(Math.PI / 2)).until(()->algaeArm.isFinished());
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

    //Main drive controlls
    rightXTrigger.whileFalse(
        DriveCommands.joystickDriveAtAngle(drive,  () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(), ()->new Rotation2d(UtilitiesFieldSectioning.getClosestSection(drive.getPose()).getRotation().getRadians()))
    ).whileTrue( DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));



    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));
    //trigger controls
    yIsPressed.whileFalse(ManipulatorStop).whileTrue(ManipulatorShoot);
    povDownisPressed.whileFalse(indexerStop).whileTrue(indexerStart);
    elevatorButtonTrigger.whileFalse(new GoToPositionElevator(elevator,0)).whileTrue(new GoToPositionElevator(elevator,2));
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
    driverController.povUp().whileTrue(drive.generatePath(new Pose2d(3.483,7.142, Rotation2d.fromDegrees(108.814))));
    driverController.povRight().whileTrue(AlgaeArmPositionSet);
    driverController.povLeft().whileTrue(new FunnelUp(funnel));
    ButtonBoardButtons.LEVEL_1.whileTrue(new GoToPositionElevator(elevator,1/4));
    ButtonBoardButtons.LEVEL_2.whileTrue(new GoToPositionElevator(elevator,2/4));
    ButtonBoardButtons.LEVEL_3.whileTrue(new GoToPositionElevator(elevator,3/4));
    ButtonBoardButtons.LEVEL_4.whileTrue(new GoToPositionElevator(elevator,4/4));
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
  public static CommandJoystick getButtonBoard() {
    return buttonboardController;
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

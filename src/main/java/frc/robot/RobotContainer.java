package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Funnel.FunnelIOReplay;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIOSim;
import frc.robot.subsystems.Manipulator.ManipulatorIOSparkMax;
import frc.robot.subsystems.Pneumatics.Pneumatics;
import frc.robot.subsystems.Pneumatics.PneumaticsIO;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
import frc.robot.subsystems.Funnel.FunnelIOSim;
import frc.robot.subsystems.Funnel.FunnelIO;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Manipulator.ManipulatorIO;
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
import frc.robot.util.ButtonBoardButtons;
import frc.robot.util.UtilitiesFieldSectioning;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.commands.GoToPositionElevator;
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
  
  // Controller
//   private final CommandXboxController testController = new CommandXboxController(2);

  //triggers
//   private final Trigger yIsPressed = new Trigger(driverController.y());
  private final Trigger povDownisPressed = new Trigger(driverController.povDown());
//   private final Trigger leftXTrigger = new Trigger(()->(Math.abs(driverController.getLeftX()))>DriveCommands.DEADBAND);
//   private final Trigger leftYTrigger = new Trigger(()->(Math.abs(driverController.getLeftY()))>DriveCommands.DEADBAND);
  private final Trigger rightXTrigger = new Trigger(()->(Math.abs(driverController.getRightX()))>DriveCommands.DEADBAND);
//   private final Trigger allTrigger = new Trigger(()->leftXTrigger.getAsBoolean() || leftYTrigger.getAsBoolean() || rightXTrigger.getAsBoolean());
  //Subsystem Definitions
  private final Drive drive;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Manipulator shooter;
  private final Indexer indexer;
  @SuppressWarnings("unused")
  private final BeamBreak beamBreakBack;
  private final BeamBreak beamBreakMid;
  // private final Funnel funnel;
  private final Elevator elevator;
  // private final AlgaeArm algaeArm;
  public static final LEDS led = new LEDS(10); //TODO: Change length based on new robot leds
  //commands
  private Command ManipulatorShoot; 
  private Command ManipulatorStop;
  @SuppressWarnings("unused")
  private Command ManipulatorClear;
  private Command indexerStart;
  private Command indexerStop;
  private Command AlgaeArmPositionSet;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber xOverride;
  
  private Command ManipulatorVariable;
  
  private Pneumatics pneumatics;

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  private ParallelRaceGroup pneumaticClimbCommand;
    
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
            shooter = new Manipulator(new ManipulatorIOSparkMax("Manipulator",ManipulatorConstants.CompBot_CONFIG) {}, ManipulatorConstants.REAL_GAINS);
            indexer = new Indexer(new IndexerIOSparkMax("Indexer",IndexerConstants.CompBot_CONFIG) {}, IndexerConstants.REAL_GAINS);
            beamBreakBack = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak1",BeamBreakConstants.CONFIG_BEAM_BREAK_1) {});
            beamBreakMid = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
            pneumatics = new Pneumatics(new PneumaticsIO() {});
          // funnel = new Funnel(new FunnelIO() {}, FunnelConstants.REAL_GAINS);
          // algaeArm = new AlgaeArm(new AlgaeArmIO() {}, AlgaeArmConstants.EXAMPLE_GAINS);
          // led = new LEDS(60);
          elevator =
              new Elevator(
                  new ElevatorIONeo("Elevator", ElevatorConstants.CompBot_CONFIG),
                 ElevatorConstants.CompBot_GAINS);
  
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
          pneumatics = new Pneumatics(new PneumaticsIO() {});
          // funnel = new Funnel(new FunnelIOSim("funnelSim", FunnelConstants.EXAMPLE_CONFIG), FunnelConstants.SIM_GAINS);
          // algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.EXAMPLE_CONFIG), AlgaeArmConstants.EXAMPLE_GAINS);
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
          pneumatics = new Pneumatics(new PneumaticsIO() {});
          // funnel = new Funnel(new FunnelIOReplay("funnelReplay"), FunnelConstants.SIM_GAINS);
          // algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.EXAMPLE_CONFIG), AlgaeArmConstants.EXAMPLE_GAINS);
          // led = new LEDS(60);
          elevator =
              new Elevator(
                  new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),ElevatorConstants.EXAMPLE_GAINS);
          break;
      }
  
      // command definitions
      ManipulatorShoot = Commands.run(()->shooter.setVelocity(10)).until(()->(!beamBreakMid.beamBreakTripped() || shooter.isFinished()));
      ManipulatorStop = Commands.run(()->shooter.setVelocity(0));

    ManipulatorClear = Commands.run(()->shooter.setVelocity(-10)).withTimeout(3).andThen(ManipulatorStop); //runs motor backwards to get rid of coral from manipulator
    // indexerStart = Commands.run(()->indexer.setVelocity(1500)).until(()->indexer.isFinished()).withTimeout(5);
    // indexerStop = Commands.run(()->indexer.setVelocity(0)).until(()->indexer.isFinished());
    // AlgaeArmPositionSet = Commands.run(()->algaeArm.setPosition(Math.PI / 2)).until(()->algaeArm.isFinished());

    //set up path planner commands
    // NamedCommands.registerCommand("AlgaeArmPosition", AlgaeArmPositionSet);
    // NamedCommands.registerCommand("ManipulatorShoot", ManipulatorShoot);
    // NamedCommands.registerCommand("IndexerStart", indexerStart);
    // NamedCommands.registerCommand("IndexerStop", indexerStop);
    // NamedCommands.registerCommand("ManipulatorStop", ManipulatorStop);
    // NamedCommands.registerCommand("ElevatorPosition", new GoToPositionElevator(elevator,1));

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



    compressor.enableDigital();
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

    //Main drive controls
    //inverted activation for testing
    // TODO:reverse the activation logic
    // rightXTrigger
    //   .whileTrue(
    //     DriveCommands.joystickDriveAtAngle(
    //       drive,  
    //       () -> -driverController.getLeftY(),
    //       () -> -driverController.getLeftX(), 
    //       () -> new Rotation2d(UtilitiesFieldSectioning.getClosestSection(drive.getPose()).getRotation().getRadians())))
    //   .whileFalse(
    //     DriveCommands.joystickDrive(
    //       drive,
    //       () -> -driverController.getLeftY(),
    //       () -> -driverController.getLeftX(),
    //       () -> -driverController.getRightX()));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()*0.5));

    //trigger controls
    // yIsPressed.whileFalse(ManipulatorStop).whileTrue(ManipulatorShoot);
    // povDownisPressed.whileFalse(indexerStop).whileTrue(indexerStart);


    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
    
    driverController.povLeft().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    driverController.a().onTrue(Commands.runOnce(() ->shooter.setVelocity(1))).onFalse(Commands.runOnce(() ->shooter.setVelocity(0)));
    driverController.b().onTrue(Commands.runOnce(() ->shooter.setVelocity(20))).onFalse(Commands.runOnce(() ->shooter.setVelocity(0)));
    driverController.leftBumper().onTrue(Commands.runOnce(() ->indexer.setVelocity(5))).onFalse(Commands.runOnce(() ->indexer.setVelocity(0)));
    driverController.rightBumper().whileTrue(DriveCommands.feedforwardCharacterization(drive));

    //Josh added a elevator zero utton
    driverController.povRight().onTrue(Commands.runOnce(() ->elevator.zeroPosition()));


    pneumaticClimbCommand = Commands.run(
        ()->pneumatics.setMode(Value.kForward))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(2)
        .andThen(()->pneumatics.setMode(Value.kReverse))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withTimeout(2);
    
    driverController.x().onTrue(pneumaticClimbCommand);


    // driverController.povRight().whileTrue(AlgaeArmPositionSet);
    // driverController.povLeft().whileTrue(new FunnelUp(funnel));

    // testController.a().whileTrue(Commands.startEnd(() ->indexer.setVoltage(4),() ->indexer.setVoltage(6)));
    // testController.b().whileTrue(Commands.startEnd(() ->shooter.setVoltage(4),() ->shooter.setVoltage(6)));
    // testController.x().whileTrue(Commands.startEnd(() ->elevator.setVoltage(testController.getLeftY()),() ->elevator.setVoltage(testController.getLeftY())));




    // driverController.x().whileTrue(Commands.runOnce(() ->elevator.setPosition(15)).ignoringDisable(true));
    // driverController.y().whileTrue(Commands.runOnce(() ->elevator.setPosition(9)).ignoringDisable(true));
    // driverController.a().whileTrue(Commands.runOnce(() ->elevator.setPosition(3)).ignoringDisable(true));
    // driverController.b().whileTrue(Commands.runOnce(() ->elevator.zeroPosition()).ignoringDisable(true));

    driverController.povUp().onTrue(Commands.runOnce(() ->elevator.incrementPosition(0.5)).ignoringDisable(true));
    driverController.povDown().onTrue(Commands.runOnce(() ->elevator.incrementPosition(-0.5)).ignoringDisable(true));



    // testController.povRight().whileTrue(Commands.startEnd(() ->indexer.setVelocity(15),() ->indexer.setVoltage(0.0)));

    // testController.povLeft().whileTrue(Commands.startEnd(() ->shooter.setVelocity(15),() ->shooter.setVoltage(0.0)));




    ButtonBoardButtons.LEVEL_1.whileTrue(new GoToPositionElevator(elevator,.25).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_2.whileTrue(new GoToPositionElevator(elevator,4).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_3.whileTrue(new GoToPositionElevator(elevator,9.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_4.whileTrue(new GoToPositionElevator(elevator,28).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_CENTER_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L3).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_RIGHT_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_RIGHT_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L6).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_RIGHT_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R6).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_RIGHT_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_CENTER_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R4).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_CENTER_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R3).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_LEFT_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.NEAR_LEFT_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.R1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_LEFT_1.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L1).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_LEFT_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.FAR_CENTER_2.whileTrue(drive.generatePath(UtilitiesFieldSectioning.L4).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // ButtonBoardButtons.LEVEL_1.whileTrue(new GoToPositionElevator(elevator, 0.25)).whileFalse(new GoToPositionElevator(elevator, 0));
    // ButtonBoardButtons.LEVEL_2.whileTrue(new GoToPositionElevator(elevator, 0.5)).whileFalse(new GoToPositionElevator(elevator, 0));
    // ButtonBoardButtons.LEVEL_3.whileTrue(new GoToPositionElevator(elevator, 0.75)).whileFalse(new GoToPositionElevator(elevator, 0));
    // ButtonBoardButtons.LEVEL_4.whileTrue(new GoToPositionElevator(elevator, 1)).whileFalse(new GoToPositionElevator(elevator, 0));
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

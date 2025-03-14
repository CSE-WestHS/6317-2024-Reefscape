package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeArmConstants;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIO;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIONeo;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIOSim;
import frc.robot.subsystems.Clamps.Clamps;
import frc.robot.subsystems.Clamps.ClampsConstants;
import frc.robot.subsystems.Clamps.ClampsIOSim;
import frc.robot.subsystems.Clamps.ClampsIOSparkMax;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerConstants;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.LEDS.LEDS;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.Manipulator.ManipulatorConstants;
import frc.robot.subsystems.Manipulator.ManipulatorIOSim;
import frc.robot.subsystems.Manipulator.ManipulatorIOSparkMax;
import frc.robot.subsystems.beam_break.BeamBreak;
import frc.robot.subsystems.beam_break.BeamBreakConstants;
import frc.robot.subsystems.beam_break.BeamBreakIODigitialInput;
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
import frc.robot.commands.*;
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
  
  // Controller
//   private final CommandXboxController testController = new CommandXboxController(2);

  //triggers
//   private final Trigger yIsPressed = new Trigger(driverController.y());
  private final Trigger povDownisPressed = new Trigger(driverController.povDown());
//   private final Trigger leftXTrigger = new Trigger(()->(Math.abs(driverController.getLeftX()))>DriveCommands.DEADBAND);
//   private final Trigger leftYTrigger = new Trigger(()->(Math.abs(driverController.getLeftY()))>DriveCommands.DEADBAND);
  private final Trigger rightXTrigger = new Trigger(()->(Math.abs(driverController.getRightX()))>DriveCommands.DEADBAND);
//   private final Trigger allTrigger = new Trigger(()->leftXTrigger.getAsBoolean() || leftYTrigger.getAsBoolean() || rightXTrigger.getAsBoolean());
  private final Trigger leftTriggerPressed = new Trigger(driverController.leftTrigger());
  //Subsystem Definitions
  private final Drive drive;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final Manipulator shooter;
  private final Indexer indexer;
  @SuppressWarnings("unused")
  private final BeamBreak beamBreakBack;
  private final BeamBreak beamBreakTop;
  private final Clamps Klamps;
  private final Elevator elevator;
  private final AlgaeArm algaeArm;
  public static final LEDS led = new LEDS(10); //TODO: Change length based on new robot leds
  //commands
  private Command ManipulatorShoot; 
  private Command ManipulatorStop;
  @SuppressWarnings("unused")
  private Command ManipulatorClear;
  private Command indexerStart;
  private Command indexerStop;
  private Command AlgaeArmPositionSet;
  private Command FeedandShoot;
  private Command faceReef;
  private Command takeOutAlgae;
  private SequentialCommandGroup SetUpShooter;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkNumber xOverride;
  
  public static boolean hasShotCoral = false;
    
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
      public RobotContainer() {
        switch (Constants.currentMode) {
          case REAL:
          System.out.println("NEW CODE");
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
            beamBreakTop = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
            // pneumatics = new Pneumatics(new PneumaticsIO() {});
          // funnel = new Funnel(new FunnelIO() {}, FunnelConstants.REAL_GAINS);
          algaeArm = new AlgaeArm(new AlgaeArmIONeo("algae arm", AlgaeArmConstants.FunnelArm_CONFIG) {}, AlgaeArmConstants.FunnelArm_GAINS);
          // led = new LEDS(60);
          Klamps = new Clamps( new ClampsIOSparkMax("Clamps", ClampsConstants.CompBot_CONFIG) , ClampsConstants.REAL_GAINS);
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
          beamBreakTop = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});
          Klamps = new Clamps(new ClampsIOSim("Klamps", ClampsConstants.EXAMPLE_CONFIG), ClampsConstants.SIM_GAINS);
          // funnel = new Funnel(new FunnelIOSim("funnelSim", FunnelConstants.EXAMPLE_CONFIG), FunnelConstants.SIM_GAINS);
          algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.FunnelArm_CONFIG), AlgaeArmConstants.FunnelArm_GAINS);
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
          beamBreakTop = new BeamBreak(new BeamBreakIODigitialInput("BeamBreak2",BeamBreakConstants.CONFIG_BEAM_BREAK_2) {});

          // funnel = new Funnel(new FunnelIOReplay("funnelReplay"), FunnelConstants.SIM_GAINS);
          algaeArm = new AlgaeArm(new AlgaeArmIOSim("AlgaeArm Sim", AlgaeArmConstants.FunnelArm_CONFIG), AlgaeArmConstants.FunnelArm_GAINS);
          // led = new LEDS(60);
          Klamps = new Clamps(new ClampsIOSim("Lamps", ClampsConstants.EXAMPLE_CONFIG), ClampsConstants.SIM_GAINS);
          elevator =
              new Elevator(
                  new ElevatorIOSim("ElevatorSim", ElevatorConstants.EXAMPLE_CONFIG),ElevatorConstants.EXAMPLE_GAINS);
          break;
      }
      //zero algae arm
      algaeArm.setArmZero();
      // command definitions
      // ManipulatorStop = Commands.run(()->shooter.setVelocity(0));
      FeedandShoot = new AllignShooterCommand(shooter, beamBreakBack,indexer);
      faceReef = DriveCommands.joystickDriveAtAngle(drive, ()->0, ()->0, ()->new Rotation2d(UtilitiesFieldSectioning.getClosestSection(drive.getPose()).getRotation().getRadians()));
      takeOutAlgae = new frc.robot.commands.AlgaeArmCommands.AlgaeArmPositionCommand(algaeArm, 0.85).withTimeout(1)
        .andThen(Commands.run(()->shooter.setVelocity(15))).withTimeout(1). andThen(new frc.robot.commands.AlgaeArmCommands.AlgaeArmPositionCommand(algaeArm,0)).withTimeout(1)
        .alongWith(Commands.run(()->shooter.setVelocity(0))).withTimeout(1);

    //set up path planner commands
    NamedCommands.registerCommand("AlgaeArmPosition", AlgaeArmPositionSet);
    NamedCommands.registerCommand("ScoreL3", (new GoToPositionElevator(elevator,27).andThen(new ShootCoral(shooter, elevator).withTimeout(1.75))));
    NamedCommands.registerCommand("ScoreL2", (new GoToPositionElevator(elevator,9.5).andThen(new ShootCoral(shooter, elevator).withTimeout(1.75))));
    NamedCommands.registerCommand("IntakeCoral", (new AllignShooterCommand(shooter, beamBreakBack, indexer)));
    NamedCommands.registerCommand("ResetElevator", (new GoToPositionElevator(elevator, 0)));
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
    algaeArm.setDefaultCommand(new AlgaeArmPositionCommand(algaeArm, 0.142));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY()*0.85,
            () -> -driverController.getLeftX()*0.85,
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
    // driverController.a().whileTrue(new ShootCoral(shooter, elevator).withTimeout(2)).whileFalse(Commands.run(()->shooter.setVelocity(0)));
    // driverController.b().onTrue(Commands.runOnce(() ->shooter.setVelocity(20))).onFalse(Commands.runOnce(() ->shooter.setVelocity(0)));
    // driverController.leftBumper().onTrue(Commands.runOnce(() ->indexer.setVelocity(5))).onFalse(Commands.runOnce(() ->indexer.setVelocity(0)));
    driverController.y().onTrue(Commands.runOnce(()->shooter.setVelocity(-1))).onFalse(Commands.runOnce(()->shooter.setVelocity(0)));
    // driverController.rightBumper().whileTrue(new IndexerToShooter(indexer, beamBreakBack)); //TODO: fix
    // driverController.povRight().whileTrue(faceReef.until(()->faceReef.isFinished()).andThen(()->System.out.println("First Command done")).andThen(()->shooter.setVelocity(100)));
    // driverController.b().whileTrue(SetUpShooter);t
    driverController.rightBumper().whileTrue(new frc.robot.commands.AlgaeArmCommands.AlgaeArmPositionCommand(algaeArm, 0));
    driverController.leftBumper().whileTrue(new frc.robot.commands.AlgaeArmCommands.AlgaeArmPositionCommand(algaeArm, 2 * Math.PI / 3));
    // driverController.rightBumper().whileTrue(drive.generatePath(new Pose2d(3.589,5.334, Rotation2d.fromDegrees(-128.721))));
    // driverController.povRight().onTrue(SetUpShooter);
    driverController.povRight().onTrue(Commands.runOnce(()->elevator.zeroPosition()).ignoringDisable(true).andThen(new GoToPositionElevator(elevator, 0)).ignoringDisable(true));
    // driverController.leftBumper().whileTrue(new AllignShooterCommand(shooter, beamBreakBack));
    // driverController.b().whileTrue(DriveCommands.joystickDriveAtAngle(drive,()->0, ()->0,()->new Rotation2d(UtilitiesFieldSectioning.getClosestSection(drive.getPose()).getRotation().getRadians())));
    // driverController.leftBumper().whileTrue(DriveCommands.feedforwardCharacterization(drive));
    driverController.b().whileTrue(new Climber(Klamps,beamBreakTop)).whileFalse(Commands.run(()->Klamps.setVoltage(0)));
    driverController.x().whileTrue(new AlgaeArmPositionCommand(algaeArm, 0.142));
    driverController.leftTrigger().whileTrue(Commands.run(()->DriveConstants.maxSpeedAt12Volts = FeetPerSecond.of(2))).whileFalse(Commands.run(()->DriveConstants.maxSpeedAt12Volts = FeetPerSecond.of(8)));
    // driverController.povUp().onTrue(Commands.runOnce(() ->elevator.incrementPosition(0.5)).ignoringDisable(true));
    // driverController.povDown().onTrue(Commands.runOnce(() ->elevator.incrementPosition(-0.5)).ignoringDisable(true));
    driverController.povUp().whileTrue(Commands.run(() -> Klamps.setVoltage(-6)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    driverController.povDown().whileTrue(Commands.run(() -> Klamps.setVoltage(10)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    


    ButtonBoardButtons.LEVEL_1.whileTrue(new GoToPositionElevator(elevator,0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_2.whileTrue(new GoToPositionElevator(elevator,4).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_3.whileTrue(new GoToPositionElevator(elevator,9.5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.LEVEL_4.whileTrue(new GoToPositionElevator(elevator,28).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    ButtonBoardButtons.FarCenterLeft.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarCenterLeft));
    ButtonBoardButtons.FarCenterRight.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarCenterRight));
    ButtonBoardButtons.NearCenterLeft.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearCenterLeft));
    ButtonBoardButtons.NearCenterRight.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearCenterRight));
    ButtonBoardButtons.FarLeftNear.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarLeftNear));
    ButtonBoardButtons.FarLeftFar.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarLeftFar));
    ButtonBoardButtons.FarRightNear.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarRightNear));
    ButtonBoardButtons.FarRightNear.whileTrue(drive.generatePath(UtilitiesFieldSectioning.FarRightFar));
    ButtonBoardButtons.NearRightFar.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearRightFar));
    ButtonBoardButtons.NearRightNear.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearRightNear));
    ButtonBoardButtons.NearLeftNear.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearLeftNear));
    ButtonBoardButtons.NearLeftFar.whileTrue(drive.generatePath(UtilitiesFieldSectioning.NearLeftFar));
    
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

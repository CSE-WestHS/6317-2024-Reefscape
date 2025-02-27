package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.ModuleIO.ModuleGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private final LoggedTunableNumber drivekP;
  private final LoggedTunableNumber turnkP;
  private final LoggedTunableNumber drivekI;
  private final LoggedTunableNumber turnkI;
  private final LoggedTunableNumber drivekD;
  private final LoggedTunableNumber turnkD;
  private final LoggedTunableNumber drivekS;
  private final LoggedTunableNumber turnkS;
  private final LoggedTunableNumber drivekV;
  private final LoggedTunableNumber turnkV;
  private final LoggedTunableNumber drivekA;
  private final LoggedTunableNumber turnkA;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Drive",
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Drive",
            "Disconnected turn motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Drive",
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);

    ModuleGains gains = io.getGains();

    drivekP = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekP", gains.drivekP());
    drivekI = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekI", gains.drivekI());
    drivekD = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekD", gains.drivekD());
    drivekS = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekS", gains.drivekS());
    drivekV = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekV", gains.drivekV());
    drivekA = new LoggedTunableNumber("Drive/" + index + "/Gains/drivekA", gains.drivekA());
    turnkP = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkP", gains.turnkP());
    turnkI = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkI", gains.turnkI());
    turnkD = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkD", gains.turnkD());
    turnkS = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkS", gains.turnkS());
    turnkV = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkV", gains.turnkV());
    turnkA = new LoggedTunableNumber("Drive/" + index + "/Gains/turnkA", gains.turnkA());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i] * DriveConstants.driveWheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          io.setGains(
              new ModuleGains(values[0], values[1], values[2], values[3], values[4], values[5],values[6],values[7],values[8],values[9],values[10],values[11]));
        },
        drivekP,
        drivekI,
        drivekD,
        drivekS,
        drivekV,
        drivekA,
        turnkP,
        turnkI,
        turnkD,
        turnkS,
        turnkV,
        turnkA);
      
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.driveWheelRadiusMeters);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}

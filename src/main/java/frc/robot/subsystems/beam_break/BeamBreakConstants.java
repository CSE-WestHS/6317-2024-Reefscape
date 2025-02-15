package frc.robot.subsystems.beam_break;

public class BeamBreakConstants {
  public record BeamBreakConfig(int id, boolean invert) {}
  /*
   *  public static final ManipulatorGains SIM_GAINS =
      new ManipulatorGains(0.6, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);
   */
  public static final BeamBreakConfig CONFIG_BEAM_BREAK_1 = new BeamBreakConfig(0, false);
  public static final BeamBreakConfig CONFIG_BEAM_BREAK_2 = new BeamBreakConfig(1, false);
}

package frc.robot.subsystems.beam_break;

public class BeamBreakConstants {
  public record BeamBreakConfig(int id, boolean invert) {}
  public static final BeamBreakConfig CONFIG_BEAM_BREAK_1 = new BeamBreakConfig(0, false);
  public static final BeamBreakConfig CONFIG_BEAM_BREAK_2 = new BeamBreakConfig(0, false);
}

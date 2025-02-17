package frc.robot.subsystems.Funnel;

public class FunnelIOReplay implements FunnelIO {
  private final String name;

  public FunnelIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}

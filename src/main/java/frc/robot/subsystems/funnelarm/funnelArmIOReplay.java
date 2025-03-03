package frc.robot.subsystems.funnelarm;

public class funnelArmIOReplay implements funnelArmIO {
  private final String name;

  public funnelArmIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}

package frc.robot.subsystems.Manipulator;

public class ManipulatorIOReplay implements ManipulatorIO {
  private final String name;

  public ManipulatorIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}

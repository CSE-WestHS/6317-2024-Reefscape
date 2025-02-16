package frc.robot.subsystems.Indexer;

public class IndexerIOReplay implements IndexerIO {
  private final String name;

  public IndexerIOReplay(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }
}

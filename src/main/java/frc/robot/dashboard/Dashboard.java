package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Dashboard {
    private final Configuration config;
    private final Tab[] tabs;

    public Dashboard(Configuration config, Tab... tabs) {
        this.config = config;
        this.tabs = tabs;
    }

    public void setup() {
        for (final var tab : tabs) {
            final var shuffleboardTab = Shuffleboard.getTab(tab.title());
            tab.displayable().addTo(shuffleboardTab, new Rect(0, 0, config.width(), config.height()));
        }
    }

    public record Configuration(int width, int height) {
    }
}

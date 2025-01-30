package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Arrays;
import java.util.Objects;

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
            tab.rootElement().addTo(shuffleboardTab, new Rect(0, 0, config.width(), config.height()));
        }
    }

    public void switchTab(String tabId) {
        // look at this beautiful one-liner
        Arrays.stream(tabs)
                .filter(tab -> Objects.equals(tab.id(), tabId))
                .findFirst()
                .ifPresent((tab -> Shuffleboard.selectTab(tab.title())));
    }

    public record Configuration(int width, int height) {
    }
}

package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public interface Element {
    void addTo(ShuffleboardContainer container, Rect rect);
}

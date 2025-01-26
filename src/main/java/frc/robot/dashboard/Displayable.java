package frc.robot.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public interface Displayable {
    void addTo(ShuffleboardContainer container, Rect rect);
}

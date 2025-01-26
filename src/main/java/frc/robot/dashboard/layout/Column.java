package frc.robot.dashboard.layout;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Displayable;
import frc.robot.dashboard.Rect;

public class Column extends Layout {
    public Column(Displayable... displayables) {
        super(displayables);
    }

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        for (int i = 0; i < displayables.length; i++) {
            final var displayable = displayables[i];
            displayable.addTo(container, rect.spliceVertically(displayables.length, i));
        }
    }
}

package frc.robot.dashboard.layout;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Element;
import frc.robot.dashboard.Rect;

public class Row extends Layout {
    public Row(Element... elements) {
        super(elements);
    }

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        for (int i = 0; i < elements.length; i++) {
            final var element = elements[i];
            element.addTo(container, rect.spliceHorizontally(elements.length, i));
        }
    }
}

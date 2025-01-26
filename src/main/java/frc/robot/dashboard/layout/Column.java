package frc.robot.dashboard.layout;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Element;
import frc.robot.dashboard.Rect;

public class Column extends Layout {
    public Column(Element... elements) {
        super(elements);
    }

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        for (int i = 0; i < elements.length; i++) {
            final var element = elements[i];
            element.addTo(container, rect.spliceVertically(elements.length, i));
        }
    }
}

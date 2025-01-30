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
        final var sizes = Rect.spliceEqually(rect.w(), elements.length);
        int x = 0;
        for (int i = 0; i < elements.length; i++) {
            final var element = elements[i];
            final var size = sizes[i];

            element.addTo(container, rect.horizontalSlice(x, size));
            x += size;
        }
    }
}

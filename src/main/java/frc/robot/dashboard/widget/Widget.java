package frc.robot.dashboard.widget;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Element;
import frc.robot.dashboard.Rect;

public abstract class Widget implements Element {
    protected final String title;

    protected Widget(String title) {
        this.title = title;
    }

    protected abstract ShuffleboardComponent<?> makeIn(ShuffleboardContainer container);

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        makeIn(container)
                .withPosition(rect.x(), rect.y())
                .withSize(rect.w(), rect.h());
    }
}

package frc.robot.dashboard.widget;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Displayable;
import frc.robot.dashboard.Rect;

import java.util.function.Supplier;

public abstract class Widget<T> implements Displayable {
    protected final String title;
    protected final Supplier<T> supplier;

    protected Widget(String title, Supplier<T> supplier) {
        this.title = title;
        this.supplier = supplier;
    }

    protected abstract ShuffleboardComponent<?> makeIn(ShuffleboardContainer container);

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        makeIn(container)
                .withPosition(rect.x(), rect.y())
                .withSize(rect.w(), rect.h());
    }
}

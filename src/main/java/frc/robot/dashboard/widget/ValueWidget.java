package frc.robot.dashboard.widget;

import java.util.function.Supplier;

public abstract class ValueWidget<T> extends Widget {
    protected final Supplier<T> supplier;

    protected ValueWidget(String title, Supplier<T> supplier) {
        super(title);
        this.supplier = supplier;
    }
}

package frc.robot.dashboard.widget;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.function.Supplier;

public class BooleanWidget extends Widget<Boolean> {
    public BooleanWidget(String title, Supplier<Boolean> supplier) {
        super(title, supplier);
    }

    @Override
    protected ShuffleboardComponent<?> makeIn(ShuffleboardContainer container) {
        return container.addBoolean(title, supplier::get)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}

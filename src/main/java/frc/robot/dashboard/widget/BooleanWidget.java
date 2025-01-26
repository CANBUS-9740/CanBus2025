package frc.robot.dashboard.widget;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

import java.util.function.Supplier;

public class BooleanWidget extends Widget<Boolean> {
    public BooleanWidget(String title, Supplier<Boolean> supplier) {
        super(title, supplier);
    }

    @Override
    protected SuppliedValueWidget<Boolean> makeIn(ShuffleboardContainer container) {
        return container.addBoolean(title, supplier::get)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}

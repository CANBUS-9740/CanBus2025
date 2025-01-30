package frc.robot.dashboard.widget;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class FieldWidget extends Widget {
    private final Field2d field;

    public FieldWidget(String title, Field2d field) {
        super(title);
        this.field = field;
    }

    @Override
    protected ShuffleboardComponent<?> makeIn(ShuffleboardContainer container) {
        return container.add(title, field)
                .withWidget(BuiltInWidgets.kField);
    }
}

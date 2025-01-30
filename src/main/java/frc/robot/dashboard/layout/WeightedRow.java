package frc.robot.dashboard.layout;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.dashboard.Element;
import frc.robot.dashboard.Rect;

public class WeightedRow extends Layout {
    private final float[] weights;

    public WeightedRow(float[] weights, Element... elements) {
        super(elements);

        if (weights.length != elements.length)
            throw new IllegalArgumentException("Weights must have the same length as the number of elements.");

        this.weights = weights;
    }

    @Override
    public void addTo(ShuffleboardContainer container, Rect rect) {
        final var sizes = Rect.spliceWeighted(rect.w(), weights);
        int x = 0;
        for (int i = 0; i < elements.length; i++) {
            final var element = elements[i];
            final var size = sizes[i];

            element.addTo(container, rect.horizontalSlice(x, size));
            x += size;
        }
    }
}

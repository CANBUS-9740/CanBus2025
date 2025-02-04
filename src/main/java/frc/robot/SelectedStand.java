package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class SelectedStand {

    public final int index;
    public final int row;
    public final Pose2d pose;

    public SelectedStand(int index, int row, Pose2d pose) {
        this.index = index;
        this.row = row;
        this.pose = pose;
    }
}

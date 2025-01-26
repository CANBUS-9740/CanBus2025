package frc.robot.dashboard.layout;

import frc.robot.dashboard.Displayable;

public abstract class Layout implements Displayable {
    protected final Displayable[] displayables;

    protected Layout(Displayable... displayables) {
        this.displayables = displayables;
    }
}

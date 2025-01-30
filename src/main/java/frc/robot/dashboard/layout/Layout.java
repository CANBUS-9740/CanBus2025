package frc.robot.dashboard.layout;

import frc.robot.dashboard.Element;

public abstract class Layout implements Element {
    protected final Element[] elements;

    protected Layout(Element... elements) {
        this.elements = elements;
    }
}

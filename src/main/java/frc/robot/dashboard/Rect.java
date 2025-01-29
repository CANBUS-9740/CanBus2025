package frc.robot.dashboard;

public record Rect(int x, int y, int w, int h) {
    public Rect spliceHorizontally(int numDivisions, int divisionIndex) {
        final var stride = (float) w / numDivisions;
        final var start = Math.round(x + stride * divisionIndex);
        final var end = Math.round(x + stride * (divisionIndex + 1));
        return new Rect(start, y, end - start, h);
    }
    public Rect spliceVertically(int numDivisions, int divisionIndex) {
        final var stride = (float) h / numDivisions;
        final var start = Math.round(y + stride * divisionIndex);
        final var end = Math.round(y + stride * (divisionIndex + 1));
        return new Rect(x, start, w, end - start);
    }
}

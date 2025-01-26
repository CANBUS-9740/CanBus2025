package frc.robot.dashboard;

public record Rect(int x, int y, int w, int h) {
    public Rect spliceHorizontally(int numDivisions, int divisionIndex) {
        final var stride = (float) w / numDivisions;
        final var start = x + divisionIndex * stride;
        final var end = start + stride;
        return new Rect(Math.round(start), y, Math.round(end - start), h);
    }
    public Rect spliceVertically(int numDivisions, int divisionIndex) {
        final var stride = (float) h / numDivisions;
        final var start = y + divisionIndex * stride;
        final var end = start + stride;
        return new Rect(x, Math.round(start), w, Math.round(end - start));
    }
}

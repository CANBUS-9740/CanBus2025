package frc.robot.dashboard;

public record Rect(int x, int y, int w, int h) {
    public Rect horizontalSlice(int newX, int newW) {
        return new Rect(newX, y, newW, h);
    }

    public Rect verticalSlice(int newY, int newH) {
        return new Rect(x, newY, w, newH);
    }

    public static int[] spliceEqually(int size, int numDivisions) {
        if (numDivisions <= 0)
            throw new IllegalArgumentException("numDivisions must be greater than 0.");

        final var sizeArray = new int[numDivisions];
        float end = 0f;
        for (int i = 0; i < numDivisions; i++) {
            float start = end;
            end += (float) size / numDivisions;
            sizeArray[i] = Math.round(end) - Math.round(start);
        }

        return sizeArray;
    }

    public static int[] spliceWeighted(int size, float[] weights) {
        if (weights.length == 0)
            throw new IllegalArgumentException("There must be at least 1 weight supplied.");

        float weightSum = 0f;
        for (final var weight : weights)
            weightSum += weight;

        if (weightSum <= 0)
            throw new IllegalArgumentException("The weight sum must be greater than 0.");

        final var sizeArray = new int[weights.length];
        float end = 0f;
        for (int i = 0; i < weights.length; i++) {
            float start = end;
            end += weights[i] / weightSum * size;
            sizeArray[i] = Math.round(end) - Math.round(start);
        }

        return sizeArray;
    }
}

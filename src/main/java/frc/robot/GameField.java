package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.dyn4j.geometry.Vector2;

import java.util.Arrays;
import java.util.Optional;

public class GameField {

    enum ReefStandSide {
        RIGHT,
        LEFT
    }

    enum SourceStandSide {
        RIGHT,
        LEFT,
        CENTER
    }

    enum ReefStand {
        STAND_1(21, 7),
        STAND_2(20, 8),
        STAND_3(19, 9),
        STAND_4(18, 10),
        STAND_5(17, 11),
        STAND_6(22, 6);

        public final int aprilTagIdBlue;
        public final int aprilTagIdRed;

        ReefStand(int aprilTagIdBlue, int aprilTagIdRed) {
            this.aprilTagIdBlue = aprilTagIdBlue;
            this.aprilTagIdRed = aprilTagIdRed;
        }
    }

    enum SourceStand {
        LEFT(13, 2),
        RIGHT(12, 1);

        public final int aprilTagIdBlue;
        public final int aprilTagIdRed;

        SourceStand(int aprilTagIdBlue, int aprilTagIdRed) {
            this.aprilTagIdBlue = aprilTagIdBlue;
            this.aprilTagIdRed = aprilTagIdRed;
        }
    }

    public static class SelectedReefStand {
        public final ReefStand stand;
        public final ReefStandSide side;
        public final Pose2d pose;

        public SelectedReefStand(ReefStand stand, ReefStandSide side, Pose2d pose) {
            this.stand = stand;
            this.side = side;
            this.pose = pose;
        }
    }

    public static class SelectedSourceStand {
        public final SourceStand stand;
        public final SourceStandSide side;
        public final Pose2d pose;

        public SelectedSourceStand(SourceStand stand, SourceStandSide side, Pose2d pose) {
            this.stand = stand;
            this.side = side;
            this.pose = pose;
        }
    }

    public static final int APRIL_TAG_PROCESSOR_BLUE = 3;
    public static final int APRIL_TAG_PROCESSOR_RED = 16;

    public static final int[] APRIL_TAGS_SOURCE_BLUE = Arrays.stream(SourceStand.values())
            .mapToInt((stand)-> stand.aprilTagIdBlue)
            .toArray();
    public static final int[] APRIL_TAGS_SOURCE_RED = Arrays.stream(SourceStand.values())
            .mapToInt((stand)-> stand.aprilTagIdRed)
            .toArray();

    public static final int[] APRIL_TAGS_REEF_BLUE = Arrays.stream(ReefStand.values())
            .mapToInt((stand)-> stand.aprilTagIdBlue)
            .toArray();
    public static final int[] APRIL_TAGS_REEF_RED = Arrays.stream(ReefStand.values())
            .mapToInt((stand)-> stand.aprilTagIdRed)
            .toArray();

    private static final double OFFSET_ON_STAND_REEF = 0.328676 / 2;
    private static final double OFFSET_ON_STAND_SOURCE = 0.328676 / 2; // TODO
    private static final double OFFSET_ROBOT = 0; // (robot length + bumpers) / 2

    private final AprilTagFieldLayout layout;

    public GameField() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public Optional<SelectedReefStand> findBestReefStandTo(Pose2d targetPose, boolean considerAngle) {
        DriverStation.Alliance alliance = getCurrentAlliance();
        double targetHeading = AngleUtils.translateAngle(targetPose.getRotation().getDegrees());

        double bestDistance = -1;
        ReefStand bestStand = null;
        ReefStandSide bestSide = null;
        Pose2d bestPose = null;

        for (ReefStand stand : ReefStand.values()) {
            for (ReefStandSide side : ReefStandSide.values()) {
                Pose2d standPose = getReefStandPoseForAlliance(stand, side, alliance);
                double distance = getDistanceToMeters(targetPose, standPose);
                double angleTo = getAngleToDegrees(targetPose, standPose);

                if ((bestDistance < 0 || distance < bestDistance) &&
                        (!considerAngle || (MathUtil.isNear(angleTo, targetHeading, RobotMap.STAND_SELECTION_HEADING_MARGIN) &&
                                MathUtil.isNear(AngleUtils.translateAngle(standPose.getRotation().getDegrees() + 180), targetHeading, RobotMap.STAND_SELECTION_GENERAL_ORIENTATION_MARGIN)))) {
                    bestDistance = distance;
                    bestStand = stand;
                    bestSide = side;
                    bestPose = standPose;
                }
            }
        }

        if (bestStand == null) {
            return Optional.empty();
        }

        return Optional.of(new SelectedReefStand(bestStand, bestSide, bestPose));
    }

    public Optional<SelectedSourceStand> getClosestSourceTo(Pose2d targetPose) {
        DriverStation.Alliance alliance = getCurrentAlliance();

        double bestDistance = -1;
        SourceStand bestStand = null;
        SourceStandSide bestSide = null;
        Pose2d bestPose = null;

        for (SourceStand stand : SourceStand.values()) {
            for (SourceStandSide side : SourceStandSide.values()) {
                Pose2d standPose = getSourceStandPoseForAlliance(stand, side, alliance);
                double distance = getDistanceToMeters(targetPose, standPose);

                if (bestDistance < 0 || distance < bestDistance) {
                    bestDistance = distance;
                    bestStand = stand;
                    bestSide = side;
                    bestPose = standPose;
                }
            }
        }

        if (bestStand == null) {
            return Optional.empty();
        }

        return Optional.of(new SelectedSourceStand(bestStand, bestSide, bestPose));
    }

    public Pose2d getPoseForReefStand(ReefStand stand, ReefStandSide side) {
        return getPoseForReefStandForAlliance(stand, side, getCurrentAlliance());
    }

    public Pose2d getPoseForReefStandForAlliance(ReefStand stand, ReefStandSide side, DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? stand.aprilTagIdRed : stand.aprilTagIdBlue;
        return getPoseForReefStandByAprilTag(id, side);
    }

    public Pose2d getPoseForReefStandByAprilTag(int id, ReefStandSide side) {
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose = calculatePoseInFrontOfAndToTheSide(
                pose,
                OFFSET_ON_STAND_REEF,
                OFFSET_ROBOT,
                side == ReefStandSide.LEFT);
        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), Rotation2d.fromDegrees(newRotation));
    }

    public Pose2d getPoseForSource(SourceStand stand, SourceStandSide side) {
        return getPoseForSourceForAlliance(stand, side, getCurrentAlliance());
    }

    public Pose2d getPoseForSourceForAlliance(SourceStand stand, SourceStandSide side, DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? stand.aprilTagIdRed : stand.aprilTagIdBlue;
        return getPoseForSourceByAprilTag(id, side);
    }

    public Pose2d getPoseForSourceByAprilTag(int id, SourceStandSide side) {
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose;
        switch (side) {
            case LEFT:
            case RIGHT:
                calculatedPose = calculatePoseInFrontOfAndToTheSide(
                        pose,
                        OFFSET_ON_STAND_SOURCE,
                        OFFSET_ROBOT,
                        side == SourceStandSide.LEFT);
                break;
            case CENTER:
                calculatedPose = calculatePoseInFrontOf(pose, OFFSET_ROBOT);
                break;
            default:
                throw new AssertionError();
        }

        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), Rotation2d.fromDegrees(newRotation));
    }

    public Pose2d getPoseToProcessor() {
        return getPoseToProcessorForAlliance(getCurrentAlliance());
    }

    public Pose2d getPoseToProcessorForAlliance(DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? APRIL_TAG_PROCESSOR_RED : APRIL_TAG_PROCESSOR_BLUE;
        return getPoseToProcessorByAprilTagId(id);
    }

    public Pose2d getPoseToProcessorByAprilTagId(int id) {
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose = calculatePoseInFrontOf(pose, OFFSET_ROBOT);
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    public Pose2d getAprilTagPose(int id) {
        return layout.getTagPose(id).orElseThrow().toPose2d();
    }

    private Pose2d getReefStandPoseForAlliance(ReefStand stand, ReefStandSide side, DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? stand.aprilTagIdRed : stand.aprilTagIdBlue;
        Pose2d pose = getAprilTagPose(id);
        return calculatePoseToTheSide(pose, OFFSET_ON_STAND_REEF, side == ReefStandSide.LEFT);
    }

    private Pose2d getSourceStandPoseForAlliance(SourceStand stand, SourceStandSide side, DriverStation.Alliance alliance) {
        int id = alliance == DriverStation.Alliance.Red ? stand.aprilTagIdRed : stand.aprilTagIdBlue;
        Pose2d pose = getAprilTagPose(id);
        return calculatePoseToTheSide(pose, OFFSET_ON_STAND_SOURCE, side == SourceStandSide.LEFT);
    }

    private Pose2d calculatePoseInFrontOfAndToTheSide(Pose2d pose, double d1, double d2, boolean isLeft) {
        double alpha = pose.getRotation().getDegrees();
        double beta = isLeft ? alpha - 90 : alpha + 90;

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 down = Vector2.create(d1, Math.toRadians(beta));
        Vector2 up = Vector2.create(d2, Math.toRadians(alpha));
        Vector2 result = start.add(down).add(up);

        return new Pose2d(result.x, result.y, pose.getRotation());
    }

    private Pose2d calculatePoseToTheSide(Pose2d pose, double d1, boolean isLeft) {
        double alpha = pose.getRotation().getDegrees();
        double beta = isLeft ? alpha - 90 : alpha + 90;

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 down = Vector2.create(d1, Math.toRadians(beta));
        Vector2 result = start.add(down);

        return new Pose2d(result.x, result.y, pose.getRotation());
    }

    private Pose2d calculatePoseInFrontOf(Pose2d pose, double d) {
        double alpha = pose.getRotation().getDegrees();

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 up = Vector2.create(d, Math.toRadians(alpha));
        Vector2 result = start.add(up);

        return new Pose2d(result.x, result.y, pose.getRotation());
    }

    public DriverStation.Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    }

    private static double getDistanceToMeters(Pose2d source, Pose2d target) {
        return Math.sqrt(Math.pow(source.getX() - target.getX(), 2) + Math.pow(source.getY() - target.getY(), 2));
    }

    private static double getAngleToDegrees(Pose2d source, Pose2d target) {
        return AngleUtils.translateAngle(Math.toDegrees(Math.atan2(target.getY() - source.getY(), target.getX() - source.getX())));
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.*;
import java.util.Optional;

public class LimeLight {

    private static final double MAX_DISTANCE_FOR_POSE = 3;

    private final String name;

    public LimeLight(String name) {
        this.name = name;
    }

    public Optional<LimelightHelpers.PoseEstimate> getPose() {
        if (!LimelightHelpers.getTV(name)) {
            SmartDashboard.putBoolean("HasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "No TV");
            return Optional.empty();
        }

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (poseEstimate.rawFiducials.length < 1) {
            SmartDashboard.putBoolean("HasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "No Fiducial");
            return Optional.empty();
        }
        if (poseEstimate.rawFiducials[0].distToRobot >= MAX_DISTANCE_FOR_POSE) {
            SmartDashboard.putBoolean("HasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "Distance");
            SmartDashboard.putNumber("Distance: ", poseEstimate.rawFiducials[0].distToRobot);

            return Optional.empty();
        }
        //boolean ZIsGood = MathUtil.isNear(0, poseEstimate., 1);
        SmartDashboard.putBoolean("HasAprilTag", true);

        return Optional.of(poseEstimate);
    }


    /*public Pose3d getPos(){
        double[] posArray = this.table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose3d(posArray[0], posArray[1], posArray[2],new Rotation3d(new Rotation2d(posArray[5])));
    }
    public boolean isGoodInitDetection( ){
        boolean ZIsGood = MathUtil.isNear(0, getPos().getZ(), 1);
        boolean XIsGood = MathUtil.isNear(360, getPos().getX(), 40) && MathUtil.isNear(0, getPos().getX(), 40);
        boolean RotationIsGood = MathUtil.isNear(90, getPos().getRotation().toRotation2d().getDegrees()%180, 8);
        return getExistanceOfContour() && ZIsGood && XIsGood && RotationIsGood;
    }
    public boolean isGoodGameDetection(Pose2d currentPos){

        boolean XIsGood = MathUtil.isNear(360, getPos().getX(), 5);
        boolean YIsGood = MathUtil.isNear(currentPos.getY(), getPos().getY(), 5);
        boolean RotationIsGood = MathUtil.isNear(currentPos.getRotation().getDegrees(), getPos().getRotation().toRotation2d().getDegrees(), 5);
        return getExistanceOfContour() && ZIsGood && XIsGood && YIsGood && RotationIsGood;
    }*/
}

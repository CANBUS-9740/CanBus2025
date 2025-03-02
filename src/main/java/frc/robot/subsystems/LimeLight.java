package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.swing.*;

public class LimeLight {
    private  final NetworkTable table;
    public LimeLight(String limeName){
        table = NetworkTableInstance.getDefault().getTable(limeName);
    }
    public Pose3d getPos(){
        double[] posArray = this.table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        return new Pose3d(posArray[0], posArray[1], posArray[2],new Rotation3d(new Rotation2d(posArray[5])));
    }
    public boolean isGoodInitDetection(Pose2d currentPos){
        boolean ZIsGood = MathUtil.isNear(0, getPos().getZ(), 1);
        boolean XIsGood = MathUtil.isNear(360, getPos().getX(), 40);
        boolean RotationIsGood = MathUtil.isNear(90, getPos().getRotation().toRotation2d().getDegrees()%180, 8);
        return getExistanceOfContour() && ZIsGood && XIsGood && RotationIsGood;
    }
    public boolean isGoodGameDetection(Pose2d currentPos){
        boolean ZIsGood = MathUtil.isNear(0, getPos().getZ(), 1);
        boolean XIsGood = MathUtil.isNear(360, getPos().getX(), 5);
        boolean YIsGood = MathUtil.isNear(currentPos.getY(), getPos().getY(), 5);
        boolean RotationIsGood = MathUtil.isNear(currentPos.getRotation().getDegrees(), getPos().getRotation().toRotation2d().getDegrees(), 5);
        return getExistanceOfContour() && ZIsGood && XIsGood && YIsGood && RotationIsGood;
    }
    public boolean getExistanceOfContour(){
        return table.getEntry("tv").getInteger(0) == 1;
    }
    public double getHorizontalOffsetAngle(){
        return table.getEntry("tx").getDouble(0);
    }
    public double getVerticalOffsetAngle(){
        return table.getEntry("ty").getDouble(0);
    }
}

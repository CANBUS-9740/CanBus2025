package frc.robot.subsystems;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class LimelightSystem extends SubsystemBase {


    public LimelightSystem() {
    }

    public Optional<LimelightHelpers.PoseEstimate> getLocation(){
        if(!LimelightHelpers.getTV("limelight")){
            return Optional.ofNullable(empty);
        } else{
            return LimelightHelpers.getBotPoseEstimate_wpiBlue();

        }
    }

}

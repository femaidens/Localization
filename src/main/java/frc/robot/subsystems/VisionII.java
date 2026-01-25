package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionII {
    private final AprilTagFieldLayout TAG_LAYOUT = 
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private PhotonCamera leftCam, rightCam;


    private final Translation3d RIGHT_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), 
            Units.inchesToMeters(-8.818), 
            Units.inchesToMeters(9));
    private final Rotation3d RIGHT_ROBOT_TO_CAM_ROT = new Rotation3d(
            0,
            0,
            0);
    private final Transform3d RIGHT_ROBOT_TO_CAM = new Transform3d(
            RIGHT_ROBOT_TO_CAM_TRANS,
            RIGHT_ROBOT_TO_CAM_ROT
        );

    private final Translation3d LEFT_ROBOT_TO_CAM_TRANS = new Translation3d(
            Units.inchesToMeters(11.248), 
            Units.inchesToMeters(8.818), 
            Units.inchesToMeters(9));
    private final Rotation3d LEFT_ROBOT_TO_CAM_ROT = new Rotation3d(
            0, 
            0, 
            0);
    private final Transform3d LEFT_ROBOT_TO_CAM = new Transform3d(
            LEFT_ROBOT_TO_CAM_TRANS,
            LEFT_ROBOT_TO_CAM_ROT
        );

    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator leftEstimator;
   
    public VisionII() {
        leftCam = new PhotonCamera("2265-ironfish");
        rightCam = new PhotonCamera("2265-greenfish");
        rightEstimator = new PhotonPoseEstimator(TAG_LAYOUT, RIGHT_ROBOT_TO_CAM);
        leftEstimator = new PhotonPoseEstimator(TAG_LAYOUT, LEFT_ROBOT_TO_CAM);
    }

    // For testing
    VisionII(PhotonCamera leftCam, PhotonCamera rightCam, PhotonPoseEstimator leftEstimator, PhotonPoseEstimator rightEstimator) {
        this.leftCam = leftCam;
        this.rightCam = rightCam;
        this.leftEstimator = leftEstimator;
        this.rightEstimator = rightEstimator;
    }

    public List<EstimatedRobotPose> getVisionUpdates(){
        List<EstimatedRobotPose> results = new ArrayList<>();
            for(var result:rightCam.getAllUnreadResults()){
                rightEstimator.estimateCoprocMultiTagPose(result).ifPresent(results::add);
            }
        
            for(var result:leftCam.getAllUnreadResults()){
                leftEstimator.estimateCoprocMultiTagPose(result).ifPresent(results::add);
            }
    
        return results;
        
    }
}

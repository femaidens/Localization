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

public class Vision {

    private final PhotonCamera leftCam;
    private final PhotonCamera rightCam;


    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator leftEstimator;
   
    /** Constructor for VisionII subsystem */
    public Vision() {
        leftCam = new PhotonCamera("2265-ironfish");
        rightCam = new PhotonCamera("2265-greenfish");
        Rotation3d rightRobotToCamRot = new Rotation3d(
                0,
                0,
                0);
        Translation3d rightRobotToCamTrans = new Translation3d(
                Units.inchesToMeters(11.248),
                Units.inchesToMeters(-8.818),
                Units.inchesToMeters(9));
        Transform3d rightRobotToCam = new Transform3d(
                rightRobotToCamTrans,
                rightRobotToCamRot
        );
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2026RebuiltWelded);
        rightEstimator = new PhotonPoseEstimator(tagLayout, rightRobotToCam);
        Rotation3d leftRobotToCamRot = new Rotation3d(
                0,
                0,
                0);
        Translation3d leftRobotToCamTrans = new Translation3d(
                Units.inchesToMeters(11.248),
                Units.inchesToMeters(8.818),
                Units.inchesToMeters(9));
        Transform3d leftRobotToCam = new Transform3d(
                leftRobotToCamTrans,
                leftRobotToCamRot
        );
        leftEstimator = new PhotonPoseEstimator(tagLayout, leftRobotToCam);
    }

    /** Constructor for VisionII subsystem, used for testing */
    Vision(PhotonCamera leftCam, PhotonCamera rightCam, PhotonPoseEstimator leftEstimator, PhotonPoseEstimator rightEstimator) {
        this.leftCam = leftCam;
        this.rightCam = rightCam;
        this.leftEstimator = leftEstimator;
        this.rightEstimator = rightEstimator;
    }

    /** Returns the estimated robot pose based on vision */
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

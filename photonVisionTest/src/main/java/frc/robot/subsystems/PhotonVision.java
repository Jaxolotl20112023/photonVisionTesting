package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 
    

    private final PhotonCamera camera; 
    private final PhotonPoseEstimator poseEstimator; 

    private List<PhotonPipelineResult> results; 
    private Transform3d tagToCamera; 
    private Transform2d tagToCameraProcessed;
    private Pose2d estimatedRobotPoseRelative; 
    private EstimatedRobotPose estimatedRobotPoseField; 
    private Transform3d cameraToRobot; 

    private Optional<PhotonPipelineResult> currentResult;

    private int tagID; 
    private String cameraName; 
    private boolean tagSeen; 

    private Field2d m_Field; 

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator; 
    

    public PhotonVision(String cameraName, CommandSwerveDrivetrain drivetrain, Transform3d cameraToRobot) {
        this.drivetrain = drivetrain;
        this.cameraName = cameraName; 

        this.camera = new PhotonCamera(cameraName); 
        this.cameraToRobot = cameraToRobot;


        poseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConst.field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);  

        
        m_Field = new Field2d();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(cameraName+" odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber(cameraName+" odometry reading x", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber(cameraName+" odometry reading y", drivetrain.getState().Pose.getY());

        SmartDashboard.putNumber(cameraName+" pigeon readings", drivetrain.getPigeon2().getYaw().getValueAsDouble());
        
        tagSeen = false; 
        results = camera.getAllUnreadResults(); 
        currentResult = Optional.empty();
        

        // look at all the tags seen
        for (var change : results)  {
            currentResult = Optional.ofNullable(change); 
        }

        // if there is no tags seen then get out of function
        if (currentResult.isEmpty() || currentResult == null) {
            SmartDashboard.putBoolean("TAG SEEN", tagSeen);
            return; 
        }

        if (!currentResult.get().hasTargets()){
            SmartDashboard.putBoolean("TAG SEEN", tagSeen);
            return; 
        } else if (currentResult.isPresent()){
            tagSeen = true; 
            // get the best tag seen 
            PhotonTrackedTarget target = currentResult.get().getBestTarget(); 

            tagToCamera = target.getBestCameraToTarget(); 

            tagID = target.getFiducialId(); 

        

            // calc the 2d pose of the robot using the field loaded and the of the tag
            Pose2d tagFieldRelativePose = (Constants.PhotonVisionConst.field_layout).getTagPose(tagID).get().toPose2d();
        
            SmartDashboard.putNumber("Current Seen Tag Field X", tagFieldRelativePose.getX());
            SmartDashboard.putNumber("Current Seen Tag Field Y", tagFieldRelativePose.getY());
            SmartDashboard.putNumber("Current Seen Tag Field Yaw", tagFieldRelativePose.getRotation().getRadians());

            tagToCameraProcessed = PhotonUtils.estimateCameraToTarget(tagToCamera.getTranslation().toTranslation2d(), 
                tagFieldRelativePose, 
                drivetrain.getRotation3d().toRotation2d());
            
            // calc the field relative position of the robot by using the tag to camera and the tag field, as well as the rotation of the camera to robot
            estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(tagToCameraProcessed, 
                tagFieldRelativePose, 
                new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), cameraToRobot.getRotation().toRotation2d()));

            if(poseEstimator.estimateCoprocMultiTagPose(currentResult.get()).isPresent()) {
                estimatedRobotPoseField = poseEstimator.estimateCoprocMultiTagPose(currentResult.get()).get();
                
                drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, Timer.getFPGATimestamp()); 
                m_Field.setRobotPose(drivetrain.getState().Pose);
            }

            
        }

        drivetrain.getModuleLocations(); 
        

        setSmartDashboard();

        
    }

    private void setSmartDashboard() {

        SmartDashboard.putData("Field", m_Field);
        // SmartDashboard.putData("Drivetrain Pose", drivetrain.getState().Pose.ge);

        SmartDashboard.putBoolean("TAG SEEN", tagSeen);

        SmartDashboard.putNumber(cameraName+" Camera ID", tagID);
        SmartDashboard.putBoolean(cameraName+" Tags Detected", currentResult.isPresent());

        SmartDashboard.putNumber(cameraName+" camera to target yaw", tagToCamera.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber(cameraName+" camera to target x", tagToCamera.getX());
        SmartDashboard.putNumber(cameraName+" camera to target y", tagToCamera.getY());

        
        SmartDashboard.putNumber(cameraName+" odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber(cameraName+" odometry reading x", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber(cameraName+" odometry reading y", drivetrain.getState().Pose.getY());

        SmartDashboard.putNumber(cameraName+" x estimated CAMERA pose", tagToCameraProcessed.getX());
        SmartDashboard.putNumber(cameraName+" y estimated CAMERA pose", tagToCameraProcessed.getY());
        SmartDashboard.putNumber(cameraName+" yaw estimated CAMERA pose", tagToCameraProcessed.getRotation().getDegrees());

        SmartDashboard.putNumber(cameraName+" x estimated robot pose", estimatedRobotPoseRelative.getX());
        SmartDashboard.putNumber(cameraName+" y estimated robot pose", estimatedRobotPoseRelative.getY());
        SmartDashboard.putNumber(cameraName+" yaw estimated robot pose", estimatedRobotPoseRelative.getRotation().getDegrees());

    }


}

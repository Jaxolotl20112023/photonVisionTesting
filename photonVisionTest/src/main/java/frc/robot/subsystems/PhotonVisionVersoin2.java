package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVisionVersoin2 extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain; 
    

    private final PhotonCamera camera; 
    private final PhotonPoseEstimator poseEstimator; 

    private List<PhotonPipelineResult> results; 
    private Transform3d tagToCamera; 
    private Transform2d tagToCameraProcessed;
    private Pose2d estimatedRobotPoseRelative; 
    private EstimatedRobotPose estimatedRobotPoseField; 
    private Transform3d cameraToRobot; 

    private int tagID; 
    private String cameraName; 
    private boolean tagSeen; 

    private Field2d m_Field; 

    private SwerveDrivePoseEstimator swerveDrivePoseEstimator; 

    private double distanceToTarget; 
    

    public PhotonVisionVersoin2(String cameraName, CommandSwerveDrivetrain drivetrain, Transform3d cameraToRobot) {
        this.drivetrain = drivetrain;
        this.cameraName = cameraName; 

        this.camera = new PhotonCamera(cameraName); 
        this.cameraToRobot = cameraToRobot;

        poseEstimator = new PhotonPoseEstimator(Constants.PhotonVisionConst.field_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);  

        distanceToTarget = 0; 

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
        Optional<PhotonPipelineResult> currentResult = Optional.empty();
        

        // look at all the tags seen
        for (var change : results)  {
            currentResult = Optional.of(change); 
        }

        // if there is no tags seen then get out of function
        if (currentResult.isEmpty() || currentResult == null) {
            tagSeen = false; 
            setSmartDashboard();
            return; 
        }

        if (!currentResult.get().hasTargets() || currentResult.isEmpty()){
            tagSeen = false;
            setSmartDashboard();
            return; 
        } else {
            tagSeen = true;
            setSmartDashboard();
            
            for (PhotonTrackedTarget target : currentResult.get().getTargets()) {
                
                // get the best tag seen 
                tagToCamera = target.getBestCameraToTarget(); 

                // get the id of the tag 
                tagID = target.getFiducialId(); 

                // get the relative position of the tag in the field
                Pose2d tagFieldRelativePose = (Constants.PhotonVisionConst.field_layout).getTagPose(tagID).get().toPose2d();
            
                // get the position of the tag relative to the field 
                SmartDashboard.putNumber("Current Seen Tag Field X", tagFieldRelativePose.getX());
                SmartDashboard.putNumber("Current Seen Tag Field Y", tagFieldRelativePose.getY());
                SmartDashboard.putNumber("Current Seen Tag Field Yaw", tagFieldRelativePose.getRotation().getRadians());

                // calculate the transformation in 2d (x,y) from the camera to the target 
                tagToCameraProcessed = PhotonUtils.estimateCameraToTarget(tagToCamera.getTranslation().toTranslation2d(), 
                    tagFieldRelativePose, 
                    drivetrain.getRotation3d().toRotation2d());
                
                // calc the field relative position of the robot by using the known distance from the tag to the camera and the known distance from the camera to the robot (translation and rotation)
                estimatedRobotPoseRelative = PhotonUtils.estimateFieldToRobot(tagToCameraProcessed, 
                    tagFieldRelativePose, 
                    new Transform2d(cameraToRobot.getTranslation().toTranslation2d(), 
                    cameraToRobot.getRotation().toRotation2d()));

                SmartDashboard.putBoolean("Did vision calculate?", true);

                // estimatedRobotPoseField = poseEstimator.estimateCoprocMultiTagPose(currentResult.get()).get();
                    
                double distanceToTarget = PhotonUtils.getDistanceToPose(estimatedRobotPoseRelative, tagFieldRelativePose); 

                drivetrain.setVisionMeasurementStdDevs(calculateStdDevs(distanceToTarget));
                drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, 
                    Timer.getFPGATimestamp(), 
                    calculateStdDevs(distanceToTarget));

                drivetrain.addVisionMeasurement(estimatedRobotPoseRelative, Timer.getFPGATimestamp()); 

                m_Field.setRobotPose(drivetrain.getState().Pose);
            }

            // setSmartDashboard();

        }
    }

    public Matrix<N3,N1> calculateStdDevs(double distance) {
        
        double baseStdDevTranslation = 0.05; // in meters
        double baseStdDevRotation = 0.1; // about 5.7 degrees

        double calculateStdDevsTrans = baseStdDevTranslation + (0.24 * Math.pow(distance,2)); 
        double calculateStdDevsRot = baseStdDevRotation + (0.43 * Math.pow(distance, 2)); 
        
        return VecBuilder.fill(calculateStdDevsTrans, calculateStdDevsTrans, calculateStdDevsRot); 
    }
 
    private void setSmartDashboard() {

        SmartDashboard.putNumber(cameraName+" odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber(cameraName+" odometry reading x", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber(cameraName+" odometry reading y", drivetrain.getState().Pose.getY());

        SmartDashboard.putBoolean("TAG SEEN", tagSeen);

        // SmartDashboard.putData("Field", m_Field);

        // SmartDashboard.putBoolean("TAG SEEN", tagSeen);

        // SmartDashboard.putNumber(cameraName+" Camera ID", tagID);

        // SmartDashboard.putNumber(cameraName+" camera to target yaw", tagToCamera.getRotation().toRotation2d().getDegrees());
        // SmartDashboard.putNumber(cameraName+" camera to target x", tagToCamera.getX());
        // SmartDashboard.putNumber(cameraName+" camera to target y", tagToCamera.getY());

        // SmartDashboard.putNumber(cameraName+" odometry readings yaw", drivetrain.getState().Pose.getRotation().getDegrees());
        // SmartDashboard.putNumber(cameraName+" odometry reading x", drivetrain.getState().Pose.getX());
        // SmartDashboard.putNumber(cameraName+" odometry reading y", drivetrain.getState().Pose.getY());

        // SmartDashboard.putNumber(cameraName+" x estimated CAMERA pose", tagToCameraProcessed.getX());
        // SmartDashboard.putNumber(cameraName+" y estimated CAMERA pose", tagToCameraProcessed.getY());
        // SmartDashboard.putNumber(cameraName+" yaw estimated CAMERA pose", tagToCameraProcessed.getRotation().getDegrees());

        // SmartDashboard.putNumber(cameraName+" x estimated robot pose", estimatedRobotPoseRelative.getX());
        // SmartDashboard.putNumber(cameraName+" y estimated robot pose", estimatedRobotPoseRelative.getY());
        // SmartDashboard.putNumber(cameraName+" yaw estimated robot pose", estimatedRobotPoseRelative.getRotation().getDegrees());

    }
}

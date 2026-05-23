// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PhotonVisionConst {

    public static final AprilTagFieldLayout field_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark); 

    public static final Transform3d orangeCameraOffset = new Transform3d(
      new Translation3d(0,0,0),
      new Rotation3d(0,0,0)
    );

    public static final Transform3d blueCameraOffset = new Transform3d(
      new Translation3d(0,0,0),
      new Rotation3d(0,0,0)
    );

  }

  public static class FieldPoseConstants { 

    public static final Pose2d hubPose = new Pose2d(
      new Translation2d(492.88,144.84), 
      new Rotation2d(0,0)
    ); 

  }

  public static class Swerve { 

    public static final Translation2d m_front_left = new Translation2d(Units.inchesToMeters(14.5),Units.inchesToMeters(14.5)); 
    public static final Translation2d m_front_right = new Translation2d(Units.inchesToMeters(14.5),Units.inchesToMeters(-14.5));
    public static final Translation2d m_back_left = new Translation2d(Units.inchesToMeters(-14.5),Units.inchesToMeters(14.5));
    public static final Translation2d m_back_right = new Translation2d(Units.inchesToMeters(-14.5),Units.inchesToMeters(-14.5));

    public static final int pigeon_id = 0; 

    public static final double kP = 0;
    public static final double kI = 0; 
    public static final double kD = 0; 

    public static final double kSwerveDeadband = 0; 

  }
}

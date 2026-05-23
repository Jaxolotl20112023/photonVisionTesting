package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.generated.TunerConstants;

public class Swerve {
    private final TalonFX m_front_left; 
    private final TalonFX m_front_right; 
    private final TalonFX m_back_left; 
    private final TalonFX m_back_right; 

    private final CANcoder front_left_canCoder; 
    private final CANcoder front_right_canCoder; 
    private final CANcoder back_left_canCoder; 
    private final CANcoder back_right_canCoder;

    private SwerveModulePosition front_left_position; 
    private SwerveModulePosition front_right_position; 
    private SwerveModulePosition back_left_position; 
    private SwerveModulePosition back_right_position;


    private double wheel_radius = TunerConstants.FrontLeft.WheelRadius; 
                    // get the circumference of the wheel / divides by the gear ratio / 2048 counts/encoder readings per rotation
    private double conversion_factor = (2*wheel_radius*Math.PI) / TunerConstants.FrontLeft.CouplingGearRatio / 2048;
    
    public Swerve() { 
        m_front_left = new TalonFX(TunerConstants.FrontLeft.DriveMotorId); 
        m_front_right = new TalonFX(TunerConstants.FrontRight.DriveMotorId); 
        m_back_left = new TalonFX(TunerConstants.BackLeft.DriveMotorId); 
        m_back_right = new TalonFX(TunerConstants.BackRight.DriveMotorId); 

        front_left_canCoder = new CANcoder(TunerConstants.FrontLeft.EncoderId);
        front_right_canCoder = new CANcoder(TunerConstants.FrontRight.EncoderId);
        back_left_canCoder = new CANcoder(TunerConstants.BackLeft.EncoderId);
        back_right_canCoder = new CANcoder(TunerConstants.BackRight.EncoderId);

    }

    public double[] getPositions() {
        return new double[] {
            m_front_left.getPosition().getValueAsDouble() * conversion_factor, 
            m_front_right.getPosition().getValueAsDouble() * conversion_factor, 
            m_back_left.getPosition().getValueAsDouble() * conversion_factor, 
            m_back_right.getPosition().getValueAsDouble() * conversion_factor
        };
    }

    public Rotation2d[] getRotations() {
        return new Rotation2d[] {
            Rotation2d.fromRadians(front_left_canCoder.getPosition().getValueAsDouble()),
            Rotation2d.fromRadians(front_right_canCoder.getPosition().getValueAsDouble()),
            Rotation2d.fromRadians(back_left_canCoder.getPosition().getValueAsDouble()),
            Rotation2d.fromRadians(back_right_canCoder.getPosition().getValueAsDouble())
        };
    }

    public SwerveModulePosition[] createModulePositions() {
        front_left_position = new SwerveModulePosition(
            getPositions()[0], 
            getRotations()[0]
        );

        front_right_position = new SwerveModulePosition(
            getPositions()[1], 
            getRotations()[1]
        );

        back_left_position = new SwerveModulePosition(
            getPositions()[2], 
            getRotations()[2]
        );

        back_right_position = new SwerveModulePosition(
            getPositions()[3], 
            getRotations()[3]
        );

        return new SwerveModulePosition[] {
            front_left_position, 
            front_right_position,
            back_left_position,
            back_right_position
        };
    }



}

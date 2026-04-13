// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Simulation extends SubsystemBase {

//     private Field2d m_Field; 
//     private CommandSwerveDrivetrain drivetrain; 

    
//     public Simulation(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain; 
//         m_Field = new Field2d(); 
//     }

//     @Override
//     public void periodic() {
//         m_Field.setRobotPose(drivetrain.getStateCopy().Pose);
//         SmartDashboard.putData("Field", m_Field);
//     }
// }

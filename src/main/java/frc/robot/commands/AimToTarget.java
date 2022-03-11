package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class AimToTarget extends CommandBase{
    private DriveSystem m_subsystem;
    private boolean m_finished = false;
    
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.7;
    final double ANGULAR_D = 0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    double targetRange = Constants.Vision.kFarTgtXPos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToTarget(DriveSystem subsystem) {
        m_subsystem = subsystem;
        m_finished = true;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public AimToTarget(DriveSystem subsystem, double i) {
        m_subsystem = subsystem;
        m_finished = true;
        targetRange =i;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        
        System.out.println("Aiming To Target");
    }

    // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         m_finished = false;
 
         PhotonPipelineResult result = m_subsystem.getCamera().getLatestResult();
 
         if (result.hasTargets()) {

             // First calculate range
 
             double range =
 
                     PhotonUtils.calculateDistanceToTargetMeters(
 
                            Units.inchesToMeters(6.70),
 
                             Constants.Vision.targetHeight,
 
                             Units.degreesToRadians(45),
 
                             Units.degreesToRadians(result.getBestTarget().getPitch()));
 
             // Use this range as the measurement we give to the PID controller.
 
             // -1.0 required to ensure positive PID controller effort _increases_ range
 
             double forwardSpeed = forwardController.calculate(range, targetRange);
             double errorForward = Math.abs(range) - Math.abs(Constants.Vision.kFarTgtXPos);
             // Also calculate angular power
 
             // -1.0 required to ensure positive PID controller effort _increases_ yaw
 
             double turnSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
             double errorTurn = Math.abs(result.getBestTarget().getYaw());
            SmartDashboard.putNumber("ef", errorForward);
            if(Math.abs(errorForward) > 0.15 || errorTurn > 5){
                m_subsystem.drive(forwardSpeed*-15.0, turnSpeed/15.0);
            }
            else{
                m_finished = true;
            }   
 
         } else {
 
             m_finished = true;
         }
 
         // Use our forward/turn speeds to control the drivetrain
     }
     @Override
     public boolean isFinished(){
         return m_finished;
     }
}

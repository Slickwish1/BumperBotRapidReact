package frc.robot.commands;

import java.util.Iterator;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

public class AimToBall extends CommandBase {
    private DriveSystem m_subsystem;
    private boolean m_finished = false;

    final double LINEAR_P = 0.25;

    final double LINEAR_D = 0.0;

    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.8;// SmartDashboard.getNumber("Angular Pos", 0.1);

    final double ANGULAR_D = 0.01;

    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private NetworkTable table;

    private boolean isBlue;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToBall(DriveSystem subsystem) {
        m_subsystem = subsystem;
        m_finished = true;
        isBlue = true;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    public AimToBall(DriveSystem subsystem, Boolean colorIsBlue ) {
        m_subsystem = subsystem;
        m_finished = true;
        isBlue = colorIsBlue;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Aiming to Ball");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ML");
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_finished = false;
        NetworkTableEntry xEntry = table.getEntry(isBlue ? "best_blue_x" : "best_red_x");
        NetworkTableEntry sizeEntry = table.getEntry(isBlue ? "best_blue_size" : "best_red_size");
        System.out.println(xEntry);
        NetworkTableEntry widthEntry = table.getEntry("width");
        double turnSpeed = 0.0;
        double linearSpeed = 0.0;
        if ((Double) xEntry.getNumber(Double.valueOf(0.0)) > Double.valueOf(1.0)) {
            Double width = (Double) widthEntry.getNumber(Double.valueOf(0));
            Double best_target_x = (Double) xEntry.getNumber(0);
            System.out.println(best_target_x);
            double errorAng = best_target_x - width / 2.0;
            m_finished = errorAng < 10;
            if (Math.abs(errorAng) > 5) {
                // System.out.println(errorAng);
                // error = error / 160.0;
                turnSpeed = turnController.calculate(errorAng, 0);
                turnSpeed = turnSpeed * (1.0 / 160.0) * 0.5;
                SmartDashboard.putNumber("turnSpeed", turnSpeed);
            }
        }
        if ((Double) sizeEntry.getNumber(Double.valueOf(0.0)) > Double.valueOf(1.0)) {
            Double size = (Double) sizeEntry.getNumber(Double.valueOf(0));
            if(Math.abs(size-28000) > 1500){
                double errorLin = size - 28000;
                linearSpeed = forwardController.calculate(errorLin/4000.0, 0.0);
            }
        }
            m_subsystem.drive(linearSpeed, turnSpeed);
    }
}
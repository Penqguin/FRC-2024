package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class operatorinterface extends SubsystemBase {
    private operatorinterface operatorinterface = null; //add comments pls
    private XboxController controller;
    private drivetrain drive = drivetrain.getInstance();
    private intake getIntake = intake.getInstance();
    private climber climb = climber.getInstance();
    private arm robotArm = arm.getInstance();

    //constructors
    private operatorinterface(){
        controller = new XboxController(0);
    }

    //Integrating Subsystem and driver inputs
    private void updateDrive(){
        drive.Drivecode(controller.getLeftY(), controller.getRightY());
    }

    private void updateIntake(){
        getIntake.runIntake(controller.getRightBumper(), controller.getLeftBumper());
    }

    private void updateClimber(){
        climb.Climber(controller.getXButton());
    }

     private void updateArm(){
        robotArm.runArm(controller.getAButton(), controller.getBButton());
    }

    @Override
    public void periodic(){
        updateDrive();
        updateIntake();
        updateClimber();
        updateArm();
    }
    
    
    public operatorinterface getInstance(){
        if (operatorinterface == null){
            operatorinterface = new operatorinterface();
        }
        return operatorinterface;
    }
}

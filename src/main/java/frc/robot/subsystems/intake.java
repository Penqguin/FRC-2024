package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class intake extends SubsystemBase {
    // Check if the driver is pressing the toggle button
    // If they aren't, don't do anything
    // If the toggle button is pressed, check if the arm state is up or down, depending on what it is, do the opposite

    private final WPI_VictorSPX RightMotor = new WPI_VictorSPX(5); // Sets up object representing the real arm motor
    private final WPI_VictorSPX LeftMotor = new WPI_VictorSPX(6); // Sets up object representing the real arm motor
    private final XboxController controller = new XboxController(0); // Sets up object for the controller

    public intake() {
        // Constructor
    }

    @Override
    public void periodic() {
        double rightTrigger = controller.getRightTriggerAxis();
        double leftTrigger = controller.getLeftTriggerAxis();

        if (rightTrigger > 0.1 && rightTrigger > leftTrigger) {
            RightMotor.set(1);
        } else if (leftTrigger > 0.1 && leftTrigger > rightTrigger) {
            LeftMotor.set(-1);
        } else {
            stopIntake(); // Stops the motor
        }
    }

    // Our Methods
    private void stopIntake() {
        RightMotor.set(0);
        LeftMotor.set(0);
    }
}
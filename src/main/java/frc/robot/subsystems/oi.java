package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class oi extends SubsystemBase {

    private static oi oi = null;
    private XboxController controller;
    private drivetrain drive;

    private oi() {
        controller = new XboxController(0);
        drive = drivetrain.getInstance();
    }

    public void updateDrive() {
        drive.Drive(controller.getLeftY(), controller.getRightX());
    }

    public void periodic() {
        updateDrive();
    }

    public static oi getInstance() {
        if (oi == null) {
            oi = new oi();
        }
        return oi;
    }
}
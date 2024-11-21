package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.xrp.XRPMotor;


public class drivetrain extends SubsystemBase {

    private static drivetrain drivetrain = null;
    private XRPMotor leftfront;
    private XRPMotor rightfront;

    private drivetrain() {
        leftfront = new XRPMotor(0);
        rightfront = new XRPMotor(1);

        leftfront.setInverted(true);
    }

    public void Drive(double Leftjoy, double Rightjoy) {
        if (Math.abs(Leftjoy) > 0.1 || Math.abs(Rightjoy) > 0.1) {
            leftfront.set(Leftjoy - Rightjoy);
            rightfront.set(Leftjoy + Rightjoy);
        } else {
            Stopdrive();
        }
    }

    private void Stopdrive() {
        leftfront.set(0);
        rightfront.set(0);
    }

    public static drivetrain getInstance() {
        if (drivetrain == null) {
            drivetrain = new drivetrain();
        }
        return drivetrain;
    }
}
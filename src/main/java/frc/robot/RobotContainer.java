package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@SuppressWarnings("unused")
public class RobotContainer {
    private Limelight ll;
    private OI oi;

    public RobotContainer() {
        ll = Limelight.getInstance();
        oi = OI.getInstance();
    }
}

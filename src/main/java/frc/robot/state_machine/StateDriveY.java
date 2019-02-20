package frc.robot.state_machine;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class StateDriveY extends StateWait {

    private MecanumDrive m_MecanumDrive;
    private double m_speed;

    public StateDriveY(MecanumDrive mecanumDrive,double speed, double nSeconds) {
        super(nSeconds);
        m_MecanumDrive = mecanumDrive;
        m_speed = speed;
    }

    @Override
    public void exit() {

    }

    @Override
    public void execute() {
        m_MecanumDrive.driveCartesian(m_speed, 0, 0);
    }

}
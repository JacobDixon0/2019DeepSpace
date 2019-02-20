package frc.robot.state_machine;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class StateFrontSoliniod extends StateWait {

    private DoubleSolenoid m_solenoid;
    private Value m_position;

    public StateFrontSoliniod(DoubleSolenoid solenoid, DoubleSolenoid.Value position , double nSeconds) {     
        super(nSeconds);
        m_solenoid = solenoid;
        m_position = position;
        
    }

    @Override
    public void exit() {

    }

    @Override
    public void execute() {
        m_solenoid.set(m_position);
    }

}
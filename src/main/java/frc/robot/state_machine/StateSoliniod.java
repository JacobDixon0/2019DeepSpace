package frc.robot.state_machine;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class StateSoliniod extends StateWait {

    private DoubleSolenoid m_solenoid;
    private Value m_position;

    public StateSoliniod(DoubleSolenoid solenoid, DoubleSolenoid.Value position , double nSeconds) {     
        super(nSeconds);
        m_solenoid = solenoid;
        m_position = position;
        
    }

    @Override
    public void execute() {
        m_solenoid.set(m_position);
    }

    @Override
    public String getName()  {
        return "StateSolinoid";
    }

}
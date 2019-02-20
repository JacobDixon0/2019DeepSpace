package frc.robot.state_machine;

import edu.wpi.first.wpilibj.Timer;

public class StateWait implements State {

    private double _nSeconds;
    private Timer timer;

    public StateWait(double nSeconds) {
        _nSeconds = nSeconds;
    }

    @Override
    public void enter() {
        timer.reset();
        timer.start();
    }

    @Override
    public void exit() {

    }

    @Override
    public void execute() {

    }

    @Override
    public Boolean isDone() {
        return timer.get() > _nSeconds;
    }

}
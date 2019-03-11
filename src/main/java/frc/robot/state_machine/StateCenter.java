package frc.robot.state_machine;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Pixy2Handler;

public class StateCenter implements State {

    private double _nSeconds;
    private Timer timer;
    private Pixy2Handler pixy;


    public StateCenter(Pixy2Handler Pixy2) {
        pixy =  Pixy2;
    }

    @Override
    public void enter() {
        
    }

    @Override
    public void exit() {

    }

    @Override
    public void execute() {
        if (pixy.y0() >= 49 & pixy.y0() <= 3){

        }
        
    }

    @Override
    public Boolean isDone() {
        return timer.get() >= _nSeconds;
    }

    @Override
    public String getName() {
        return "StateWait";
    }

}
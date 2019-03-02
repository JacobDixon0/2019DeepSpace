package frc.robot.state_machine;

import java.util.Vector;

public class StateMachine implements State {

    private Vector<State> states;
    private int index;
    private boolean firstRun;

    public StateMachine() {
        states = new Vector<State>();
        reset();
    }

    public void addState(State s) {
        states.add(s);
    }

    @Override
    public void enter() { }

    @Override
    public void exit() { }

    @Override
    public void execute() {
        if (index < states.size()) {
            State currentState = states.get(index);

            // handle initial case
            if (firstRun) { 
                currentState.enter();
                firstRun = false;
            }

            currentState.execute();

            if (currentState.isDone()) {
                states.get(index).exit();
                index += 1;
                
                // handle last case
                if (index < states.size()) {  
                    states.get(index).enter();
                }                        
            }
        }
    }

    @Override
    public Boolean isDone() {
        return index >= states.size();
    }
    public void reset() {
        index = 0;
        firstRun = true;
    }
}
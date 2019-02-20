package frc.robot.state_machine;

import java.util.Vector;

public class StateMachine implements State {

    private Vector<State> states;
    private int index;

    StateMachine() {
        reset();
    }

    public void addState(State s) {
        states.add(s);
    }

    @Override
    public void enter() {
        reset();
    }

    @Override
    public void exit() {

    }

    @Override
    public void execute() {
        if (index < states.size()) {
            State currentState = states.get(index);
            currentState.execute();
            if (currentState.isDone()) {
                states.get(index).exit();
                index += 1;
                states.get(index).enter();
            }
        }
    }

    @Override
    public Boolean isDone() {
        return index >= states.size();
    }
    public void reset() {
        index = 0;
    }
}
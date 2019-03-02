package frc.robot.state_machine;

import java.util.Vector;

public class StateGroup implements State {

    private Vector<State> group;

    public StateGroup() {
        group = new Vector<State>();
    }

    public void addState(State state) {
        group.add(state);
    }

	@Override
	public void enter() {
		for (State s : group) {
            s.enter();
        }
	}

	@Override
	public void exit() {
		for (State s : group) {
            s.exit();
        }
	}

	@Override
	public void execute() {
		for (State s : group) {
            if (!s.isDone()) {
                s.execute();
            }
        }
	}

	@Override
	public Boolean isDone() {
        boolean allDone = true;
        for (State s : group) {
            if (!s.isDone()) {
                allDone = false;
            }
        }
        return allDone;
	}
}
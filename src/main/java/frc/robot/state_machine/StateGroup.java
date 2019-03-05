package frc.robot.state_machine;

import java.util.Vector;
import java.util.stream.Collectors;

public class StateGroup implements State {

    private Vector<State> group;
    private String stateName;
    private String name;
    private int stateNumber;
    

    public StateGroup(String name ) {
        this.name = name;
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
    
    @Override
    public String getName() {    
        String groupNames = group.stream().map(s -> s.getName()).collect(Collectors.joining(", "));
        return name + "(" + groupNames + ")";
    }
   
}
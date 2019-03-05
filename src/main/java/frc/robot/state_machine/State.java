package frc.robot.state_machine;

public interface State {

    public void enter();
    public void exit();
    public void execute();
    public Boolean isDone();
    public String getName();

}
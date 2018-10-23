package ca.mcgill.ecse211.Gyro;

public interface GyroController {
	
	public boolean isRunning();
	
	public Object getLock();
	
	public void process(float value);
	
}

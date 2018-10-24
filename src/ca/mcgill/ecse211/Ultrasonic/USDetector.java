package ca.mcgill.ecse211.Ultrasonic;

import lejos.hardware.Sound;

public class USDetector implements UltrasonicController {

	public final int FILTER_OUT = 20;
	public int filterControl;
	public static volatile boolean running;
	public static boolean objectDetected;
	
	
	public static Object lock;
	
	public USDetector() {
		this.filterControl = 0;
		running = true;
		objectDetected = false;
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean isRunning() {
		// TODO Auto-generated method stub
		return running;
	}

	@Override
	public Object getLock() {
		// TODO Auto-generated method stub
		return lock;
	}

	@Override
	public void process(int value) {
		if (ringDetected(value)) {
			objectDetected = true;
			Sound.beepSequence();
			
		}
	}
	
	public boolean ringDetected(int distance) {
		if (distance <= 10 && filterControl < FILTER_OUT) {
			filterControl++;
			return false;
		} else if (distance <= 10) {
			filterControl = 0;
			return true;
		} else {
			filterControl = 0;
			return false;
		}
	}

}

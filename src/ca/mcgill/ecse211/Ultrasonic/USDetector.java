package ca.mcgill.ecse211.Ultrasonic;

import ca.mcgill.ecse211.Lab5.Navigation;
import lejos.hardware.Sound;

public class USDetector implements UltrasonicController {

	public final int FILTER_OUT = 20;
	public int filterControl;
	public static volatile boolean running;
	
	
	public static Object lock;
	
	public USDetector() {
		this.filterControl = 0;
		running = true;
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
			Sound.beepSequence();
//			Object lock = new Object();
//			USDetector.lock = lock;
//			Navigation.lock	= lock;
//			synchronized(lock) {
//				try {
//					lock.wait();
//					Thread.sleep(2000);
//				} catch (InterruptedException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			}
			
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

package ca.mcgill.ecse211.Gyro;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class AngleSampler implements GyroController {

	public boolean running;
	private float theta;

	public static Lock lock = new ReentrantLock(true); // Fair lock for
	// concurrent writing
	private volatile boolean isReseting = false; // Indicates if a thread is
	// trying to reset any
	// position parameters
	private Condition doneReseting = lock.newCondition(); // Let other threads
	// know that a reset
	// operation is
	// over.

	public AngleSampler() {
		this.running = true;
		
	}

	public float getTheta() {
		float theta = 0;
	    lock.lock();
	    try {
	      while (isReseting) { // If a reset operation is being executed, wait
	        // until it is over.
	        doneReseting.await(); 
	        // Using await() is lighter on the CPU
	        // than simple busy wait.
	      }
	     theta = this.theta;
	    } catch (InterruptedException e) {
	      // Print exception to screen
	      e.printStackTrace();
	    } finally {
	      lock.unlock();
	    }
		return theta;
	}

	public void setTheta(float theta) {

		lock.lock();
		isReseting = true;
		try {
			this.theta = theta;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
		} finally {
			lock.unlock();
		}
	}

	@Override
	public boolean isRunning() {
		
		return this.running;
	}

	@Override
	public Object getLock() {
		
		return null;
	}


	@Override
	public void process(float value) {
		
		this.setTheta(-value);
		

	}

}

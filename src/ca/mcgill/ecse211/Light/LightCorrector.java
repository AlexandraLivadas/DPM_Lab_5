package ca.mcgill.ecse211.Light;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import ca.mcgill.ecse211.Lab5.Lab5;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;


public class LightCorrector implements LightController {

	private Odometer odometer;
	public boolean running = true;

	public static float firstReading = -1;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 11.6; //in cm, 4.5inches

	private double corrX;
	private double corrY;

	public static final double TILE_SIZE = Lab5.TILE_SIZE;
	public static final double HALF_TILE_SIZE = Lab5.TILE_SIZE/2;
	private static final float ERROR_THRESHOLD = 5.0f;

	public static Lock lock = new ReentrantLock(true); // Fair lock for
	// concurrent writing
	private volatile boolean isReseting = false; 
	// Indicates if a thread is trying to reset any position parameters
	private Condition doneReseting = lock.newCondition(); // Let other threads
	// know that a reset operation is over

	public LightCorrector() {
		corrX = 0;
		corrY = 0;
		try {
			odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			
			e.printStackTrace();
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

	public void correct() {
		lock.lock();
		try {
			while (isReseting) { // If a reset operation is being executed, wait
				// until it is over.
				doneReseting.await(); 
				// Using await() is lighter on the CPU
				// than simple busy wait.
			}	
			odometer.update(corrX, corrY, 0);
			corrX = 0;
			corrY = 0;


		} catch (InterruptedException e) {
			// Print exception to screen
			e.printStackTrace();
		} finally {
			lock.unlock();
		}

	}

	@Override
	public void process(int value) {
		double xyt[] = odometer.getXYT();

		if (firstReading == -1) { //Set the first reading value
			firstReading = value;
		}

		if ((100*Math.abs(value - firstReading)/firstReading) > lightThreshold) {

			double lineX, lineY, errorX, errorY, deltaX, deltaY;

			deltaX = Math.sin(Math.toRadians(xyt[2])) * sensorDistance;
			deltaY = Math.cos(Math.toRadians(xyt[2])) * sensorDistance;
			lineX = xyt[0] - deltaX;
			lineY = xyt[1] - deltaY;
			
			errorX = lineX % TILE_SIZE;
			if (errorX >= HALF_TILE_SIZE) {
				errorX -= TILE_SIZE;
			} else if (errorX <= -HALF_TILE_SIZE) {
				errorX += TILE_SIZE;
			}
			
			errorY = lineY % TILE_SIZE;
			if (errorY >= HALF_TILE_SIZE) {
				errorY -= TILE_SIZE;
			} else if (errorY <= -HALF_TILE_SIZE) {
				errorY += TILE_SIZE;
			}
			
			if (Math.abs(errorX) <= ERROR_THRESHOLD && errorX <= errorY) {
			

				lock.lock();
				isReseting = true;
				try {
					corrX = -errorX;
					isReseting = false; // Done reseting
					doneReseting.signalAll(); // Let the other threads know that you are
				} finally {
					lock.unlock();
				}

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					
					e.printStackTrace();
				}

			} else if (Math.abs(errorY) <= ERROR_THRESHOLD) {
				// probably y line
			

				lock.lock();
				isReseting = true;
				try {
					corrY = -errorY;
					isReseting = false; // Done reseting
					doneReseting.signalAll(); // Let the other threads know that you are
				} finally {
					lock.unlock();
				}

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					
					e.printStackTrace();
				}

			}

		}

	}

}

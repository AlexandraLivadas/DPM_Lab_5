package ca.mcgill.ecse211.Light;

import java.text.DecimalFormat;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import ca.mcgill.ecse211.Lab5.Lab5;
import ca.mcgill.ecse211.Lab5.Navigation;
import ca.mcgill.ecse211.Light.LightLocalizer.LocalizationState;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class LightCorrector implements LightController {

	private Odometer odometer;


	public boolean running = true;

	public static float firstReading = -1;

	private static final float lightThreshold = 20.0f;
	private static final double sensorDistance = 11.3; //in cm, 4.5inches


	private double corrX;
	private double corrY;
	private boolean update;

	public static final double TILE_SIZE = Lab5.TILE_SIZE;
	private static final float ERROR_THRESHOLD = 4.0f;

	public static Lock lock = new ReentrantLock(true); // Fair lock for
	// concurrent writing
	private volatile boolean isReseting = false; // Indicates if a thread is
	// trying to reset any
	// position parameters
	private Condition doneReseting = lock.newCondition(); // Let other threads
	// know that a reset
	// operation is
	// over.

	public LightCorrector() {
		corrX = 0;
		corrY = 0;
		try {
			odometer = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean isRunning() {
		// TODO Auto-generated method stub
		return this.running;
	}

	@Override
	public Object getLock() {
		// TODO Auto-generated method stub
		return null;
	}

	public void correct() {
		lock.lock();
		try {
			while (isReseting) { // If a reset operation is being executed, wait
				// until it is over.
				doneReseting.await(); // Using await() is lighter on the CPU
				// than simple busy wait.
			}	
//			TextLCD lcd = LocalEV3.get().getTextLCD();
//			DecimalFormat numberFormat = new DecimalFormat("######0.00");
//			lcd.drawString("CX: " + numberFormat.format(corrX), 0, 5);
//			lcd.drawString("CY: " + numberFormat.format(corrY), 0, 6);
//
//			while (Button.waitForAnyPress() != Button.ID_ENTER);

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
			errorY = lineY % TILE_SIZE;

			if (Math.abs(errorX) <= ERROR_THRESHOLD && errorX <= errorY) {
				Sound.beepSequence();

				lock.lock();
				isReseting = true;
				try {
					corrX = deltaX - (xyt[0] % TILE_SIZE);
					if (Math.abs(corrX) > sensorDistance) {
						corrX = 0;
					}
					isReseting = false; // Done reseting
					doneReseting.signalAll(); // Let the other threads know that you are
				} finally {
					lock.unlock();
				}

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			} else if (Math.abs(errorY) <= ERROR_THRESHOLD) {
				// probably y line
				Sound.beepSequence();

				lock.lock();
				isReseting = true;
				try {
					corrY = deltaY - (xyt[1] % TILE_SIZE);
					if (Math.abs(corrY) > sensorDistance) {
						corrY = 0;
					}
					isReseting = false; // Done reseting
					doneReseting.signalAll(); // Let the other threads know that you are
				} finally {
					lock.unlock();
				}

				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

			}

		}

	}

}

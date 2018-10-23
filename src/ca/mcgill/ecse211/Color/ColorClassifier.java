package ca.mcgill.ecse211.Color;

import ca.mcgill.ecse211.Ultrasonic.USLocalizer;

import java.util.ArrayList;

import ca.mcgill.ecse211.Lab5.Navigation;
import ca.mcgill.ecse211.Odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;

public class ColorClassifier implements ColorController{

	private Odometer odo;
	private Navigation nav;

	public enum ClassificationState{INIT, CLASSIFYING, DONE};

	public enum RingColors{BLUE, GREEN, YELLOW, ORANGE};

	private double lightThreshold = 20.0;
	public float lightSensorIntensity;
	private double sensorDistance = 11.3; //in cm, 4.5inches
	private final double WHEEL_RAD = 2.2;
	
	private ArrayList<RingColors> detectedRings;
	
	public static RingColors targetRing;
	public static boolean targetDetected;
	public static RingColors detectedRing;

	public static double[] blueRValues = {0.191671, 0.026218};
	public static double[] blueGValues = {0.593177, 0.043177};
	public static double[] blueBValues = {0.7793101, 0.0390857};

	public static double[] greenRValues = {0.504023, 0.007849};
	public static double[] greenGValues = {0.845422, 0.003544};
	public static double[] greenBValues = {0.1757721, 0.0158844};

	public static double[] orangeRValues = {0.97871, 0.007473};
	public static double[] orangeGValues = {0.181839, 0.036419};
	public static double[] orangeBValues = {0.0865479, 0.0137534};

	public static double[] yellowRValues = {0.883695, 0.016992};
	public static double[] yellowGValues = {0.446074, 0.033467};
	public static double[] yellowBValues = {0.1366481, 0.0045489};

	public static Object lock;
	public boolean running;

	

	public ClassificationState state = ClassificationState.INIT;


	public ColorClassifier(Odometer odo, Navigation nav, RingColors targetRing) {
		this.odo = odo;
		this.nav = nav;
		this.running = true;
		this.detectedRings = new ArrayList<RingColors>();
		ColorClassifier.targetRing = targetRing;
		
	}

	public void process(float[] values) {
		RingColors ring = detectColor(values);
		if (ring != null && detectedRings.indexOf(ring) == -1) {
			detectedRings.add(ring);
			ColorClassifier.detectedRing = ring;
			
			if (ring == targetRing) {
				Sound.beep();

				// stopping navigation
				Object lock = new Object();
				Navigation.lock = lock;
				nav.setRunning(false);
				nav.clearCoordList();

				Navigation.lock = null;
				synchronized(lock) {
					lock.notifyAll();
				}
				// stopping this thread
				this.running = false;
			} else {
				Sound.twoBeeps();
			}
			
		}
	}


	public RingColors detectColor(float[] values) {
		float R, G, B;
		R = values[0];
		G = values[1];
		B = values[2];

		if (withinGaussDist(R, blueRValues, 2) &&
				withinGaussDist(G, blueGValues, 2) &&
				withinGaussDist(B, blueBValues, 2)) {
			return RingColors.BLUE;
		} else if (withinGaussDist(R, greenRValues, 2) &&
				withinGaussDist(G, greenGValues, 2) &&
				withinGaussDist(B, greenBValues, 2)) {
			return RingColors.GREEN;
		} else if (withinGaussDist(R, orangeRValues, 2) &&
				withinGaussDist(G, orangeGValues, 2) &&
				withinGaussDist(B, orangeBValues, 2)) {
			return RingColors.ORANGE;
		} else if (withinGaussDist(R, yellowRValues, 2) &&
				withinGaussDist(G, yellowGValues, 2) &&
				withinGaussDist(B, yellowBValues, 2)) {
			return RingColors.YELLOW;
		}
		return null;
	}


	public boolean withinGaussDist(double value, double[] target, int sigma) {
		return (Math.abs(value - target[0]) <= sigma * target[1]);
	}

	public boolean isRunning() {
		return this.running;
	}

	public Object getLock() {
		return lock;
	}


}

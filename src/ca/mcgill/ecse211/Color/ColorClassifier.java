package ca.mcgill.ecse211.Color;

import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.Lab5.Lab5.RingColors;
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

	private int lineCount;

	public enum ClassificationState{INIT, CLASSIFYING, DONE};
	
	private double thetaX, thetaY;
	private double correctedX, correctedY, correctedTheta;
	private double deltaThetaX, deltaThetaY, deltaTheta;
	private float firstReading;
	private double lightThreshold = 20.0;
	public float lightSensorIntensity;
	private double sensorDistance = 11.3; //in cm, 4.5inches
	private final double WHEEL_RAD = 2.2;
	private double[] lineAngles, linePos;
	private static final int ROTATE_SPEED = 100;
	private static RingColors targetRing;
	private static double[] targetR;
	private static double[] targetB;
	private static double[] targetG;
	public static boolean targetDetected;
	public static RingColors detectedRing;

	double[] blueRValues = {0.191671, 0.026218};
	double[] blueGValues = {0.593177, 0.043177};
	double[] blueBValues = {0.7793101, 0.0390857};
	
	double[] greenRValues = {0.504023, 0.007849};
	double[] greenGValues = {0.845422, 0.003544};
	double[] greenBValues = {0.1757721, 0.0158844};
	
	double[] orangeRValues = {0.97871, 0.007473};
	double[] orangeGValues = {0.181839, 0.036419};
	double[] orangeBValues = {0.0865479, 0.0137534};
	
	double[] yellowRValues = {0.883695, 0.016992};
	double[] yellowGValues = {0.446074, 0.033467};
	double[] yellowBValues = {0.1366481, 0.0045489};
	
	public static Object lock;
	public boolean running;


	public ClassificationState state = ClassificationState.INIT;


	public ColorClassifier(Odometer odo, Navigation nav, RingColors targetRing) {
		this.odo = odo;
		this.nav = nav;
		this.firstReading = -1;
		this.lineCount = 0;
		this.lineAngles = new double[4];
		this.linePos = new double[3];
		this.running = true;
		this.targetRing = targetRing;
	}

	public void process(float[] values) {
		switch(state) {
		case INIT:
			
			//Set up all possible values
			//{redMi, redS, greenMi, greenS, blueMi, blueS}
//			double[] blueRValues = {0.191671, 0.026218};
//			double[] blueGValues = {0.593177, 0.043177};
//			double[] blueBValues = {0.7793101, 0.0390857};
//			
//			double[] greenRValues = {0.504023, 0.007849};
//			double[] greenGValues = {0.845422, 0.003544};
//			double[] greenBValues = {0.1757721, 0.0158844};
//			
//			double[] orangeRValues = {0.97871, 0.007473};
//			double[] orangeGValues = {0.181839, 0.036419};
//			double[] orangeBValues = {0.0865479, 0.0137534};
//			
//			double[] yellowRValues = {0.883695, 0.016992};
//			double[] yellowGValues = {0.446074, 0.033467};
//			double[] yellowBValues = {0.1366481, 0.0045489};
			
			//Set up target RGB values
			if (targetRing.equals(RingColors.BLUE)) {
				targetR = blueRValues;
				targetG = blueGValues;
				targetB = blueBValues;
			}
			if (targetRing.equals(RingColors.GREEN)) {
				targetR = greenRValues;
				targetG = greenGValues;
				targetB = greenBValues;
			}
			if (targetRing.equals(RingColors.ORANGE)) {
				targetR = orangeRValues;
				targetG = orangeGValues;
				targetB = orangeBValues;
			}
			if (targetRing.equals(RingColors.YELLOW)) {
				targetR = yellowRValues;
				targetG = yellowGValues;
				targetB = yellowBValues;
			}
			

		case CLASSIFYING:

			//Is it Blue?
			if ((values[0] <= (blueRValues[0] + 2*blueRValues[1]) || values[0] >= (blueRValues[0] - 2*blueRValues[1]))
					&& (values[1] <= (blueGValues[0] + 2*blueGValues[1]) || values[1] >= (blueGValues[0] - 2*blueGValues[1]))
					&& (values[2] <= (blueBValues[0] + 2*blueBValues[1]) || values[0] >= (blueBValues[0] - 2*blueBValues[1]))) {
				detectedRing = RingColors.BLUE;
				//TODO send this to the display
				if (detectedRing.equals(targetRing)) {
					targetDetected = true;
					//TODO stop navigating and go to upper point
				}
				else
					targetDetected = false;
			}
			//Is it Green?
			if ((values[0] <= (greenRValues[0] + 2*greenRValues[1]) || values[0] >= (greenRValues[0] - 2*greenRValues[1]))
					&& (values[1] <= (greenGValues[0] + 2*greenGValues[1]) || values[1] >= (greenGValues[0] - 2*greenGValues[1]))
					&& (values[2] <= (greenBValues[0] + 2*greenBValues[1]) || values[0] >= (greenBValues[0] - 2*greenBValues[1]))) {
				detectedRing = RingColors.GREEN;
				//TODO send this to the display
				if (detectedRing.equals(targetRing)) {
					targetDetected = true;
					//TODO stop navigating and go to upper point
				}
				else
					targetDetected = false;
			}
			//Is it Orange?
			if ((values[0] <= (orangeRValues[0] + 2*orangeRValues[1]) || values[0] >= (orangeRValues[0] - 2*orangeRValues[1]))
					&& (values[1] <= (orangeGValues[0] + 2*orangeGValues[1]) || values[1] >= (orangeGValues[0] - 2*orangeGValues[1]))
					&& (values[2] <= (orangeBValues[0] + 2*orangeBValues[1]) || values[0] >= (orangeBValues[0] - 2*orangeBValues[1]))) {
				detectedRing = RingColors.ORANGE;
				//TODO send this to the display
				if (detectedRing.equals(targetRing)) {
					targetDetected = true;
					//TODO stop navigating and go to upper point
				}
				else
					targetDetected = false;
			}
			//Is it Yellow?
			if ((values[0] <= (yellowRValues[0] + 2*yellowRValues[1]) || values[0] >= (yellowRValues[0] - 2*yellowRValues[1]))
					&& (values[1] <= (yellowGValues[0] + 2*yellowGValues[1]) || values[1] >= (yellowGValues[0] - 2*yellowGValues[1]))
					&& (values[2] <= (yellowBValues[0] + 2*yellowBValues[1]) || values[0] >= (yellowBValues[0] - 2*yellowBValues[1]))) {
				detectedRing = RingColors.YELLOW;
				//TODO send this to the display
				if (detectedRing.equals(targetRing)) {
					targetDetected = true;
					//TODO stop navigating and go to upper point
				}
				else
					targetDetected = false;
			}
			
		case DONE:
			this.running = false;
		default:
			break;
		}
		
	}
	
	public boolean isRunning() {
		return this.running;
	}

	public Object getLock() {
		return lock;
	}
	
	
}

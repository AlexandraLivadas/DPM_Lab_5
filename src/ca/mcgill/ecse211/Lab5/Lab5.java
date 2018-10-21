package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.Ultrasonic.USDetector;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.Light.LightLocalizer;
import ca.mcgill.ecse211.Light.LightPoller;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.UARTSensor;
import lejos.robotics.SampleProvider;

public class Lab5 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port usPort2 = LocalEV3.get().getPort("S2");
	private static final Port lsPort = LocalEV3.get().getPort("S1");

	//Setting up ultrasonic sensor
	public static UARTSensor usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");

	//Setting up ultrasonic sensor 2
	public static UARTSensor usSensor2 = new EV3UltrasonicSensor(usPort2);
	public static SampleProvider usValue2 = usSensor2.getMode("Distance");
	
	//Setting up light sensor

	public static UARTSensor lsSensor = new EV3ColorSensor(lsPort);
	public static SampleProvider lsValue = lsSensor.getMode("Red");

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 10.55;
	public static final double TILE_SIZE = 30.48;
	
	public enum RingColors{BLUE, GREEN, YELLOW, ORANGE};
	
	public static final int startOption = 3;
	public static final double[] startCorner = {1.0, 1.0};
	public static final double[] endCorner = {3.0, 3.0};
	public static final RingColors targetRing = RingColors.BLUE;

	
	

	public static void main(String[] args) throws OdometerExceptions {
		// init thread to exit application
		Thread exitThread = new Thread() {
			public void run() {
				while (Button.waitForAnyPress() != Button.ID_ESCAPE);
				System.exit(0);
			}
		};
		exitThread.start();


		int buttonChoice;

		//Setting up the odometer and display
		Odometer odo = Odometer.getOdometer(leftMotor, rightMotor, WHEEL_BASE, WHEEL_RAD);
		Navigation nav = new Navigation(odo, leftMotor, rightMotor, WHEEL_RAD, WHEEL_BASE, TILE_SIZE);
		Display display = new Display(lcd);
		USLocalizer USLocal = new USLocalizer(odo);
		UltrasonicPoller usPoller = new UltrasonicPoller(usValue, USLocal);
		LightLocalizer LSLocal = new LightLocalizer(odo, nav);
		LightLocalizer.lock = USLocalizer.done;
		LightPoller lsPoller = new LightPoller(lsValue, LSLocal);
		
			

		


		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Falling| Rising ", 0, 2);
			lcd.drawString(" edge  |   edge ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) { //US Localization has been selected
			// clear the display
			lcd.clear();
			USLocal.setType(LocalizationType.FALLING_EDGE);
		} else { 
			// clear the display
			lcd.clear();
			USLocal.setType(LocalizationType.RISING_EDGE);
		}
		
		// Start odometer and display threads
		Thread odoThread = new Thread(odo);
		odoThread.start();
		Thread displayThread = new Thread(display);
		displayThread.start();
//		Thread usPollerThread = new Thread(usPoller);
//		usPollerThread.start();
//		Thread lsPollerThread = new Thread(lsPoller);
//		lsPollerThread.start();
//		
//		try {
//			usPollerThread.join();
//			lsPollerThread.join();
//			usSensor.close();
//			lsSensor.close();
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		
//		// set position according to startOption
//		switch(startOption) {
//		case 1:
//			odo.setXYT(6*TILE_SIZE, 0, 270);
//			break;
//		case 3:
//			odo.setXYT(0, 6*TILE_SIZE, 90);
//			break;
//		case 2:
//			odo.setXYT(6*TILE_SIZE, 6*TILE_SIZE, 180);
//			break;
//		}
//		double[] xyt = odo.getXYT();
//		
//		nav.travelTo(xyt[0]/TILE_SIZE, startCorner[1] - 1);
//		nav.travelTo(startCorner[0] - 1, startCorner[1] - 1);
//		Thread navThread  = new Thread(nav);
//		navThread.start();
//		
//		try {
//			navThread.join();
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		
//		nav.turnTo(odo.getXYT()[2], 0);
		
		
		// find ring part
		
		// spiral code
		USDetector USDetect = new USDetector();
		UltrasonicPoller usPoller2 = new UltrasonicPoller(usValue2, USDetect);
		
		
		
		initSpiral(nav, startCorner, endCorner);
		Thread navThread = new Thread(nav);
		navThread.start();
		Thread usPollerThread2 = new Thread(usPoller2);
		usPollerThread2.start();
		
		
		try {
			navThread.join();
			usPollerThread2.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		

	}
	
	public static void initSpiral(Navigation nav, double[] Ll, double[] Rr) {
		double x, X, y, Y;
		
		x = Ll[0] - 0.5;
		X = Rr[0] + 0.5;
		y = Ll[1] - 0.5;
		Y = Rr[1] + 0.5;
		
		nav.travelTo(x, y);
		while(X>x && Y>y) {
			nav.travelTo(x, Y);
			x++;
			nav.travelTo(X, Y);
			Y--;
			nav.travelTo(X, y);
			X--;
			nav.travelTo(x, y);
			y++;	
		}
	}
}

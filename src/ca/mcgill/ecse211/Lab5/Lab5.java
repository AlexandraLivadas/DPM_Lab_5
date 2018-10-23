package ca.mcgill.ecse211.Lab5;

import ca.mcgill.ecse211.Ultrasonic.USDetector;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer.LocalizationType;
import ca.mcgill.ecse211.Ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Color.ColorClassifier.RingColors;
import ca.mcgill.ecse211.Gyro.AngleSampler;
import ca.mcgill.ecse211.Gyro.GyroPoller;
import ca.mcgill.ecse211.Color.ColorPoller;
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
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.UARTSensor;
import lejos.robotics.SampleProvider;

public class Lab5 {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S4");
	private static final Port gyroPort = LocalEV3.get().getPort("S2");
	private static final Port lsPort = LocalEV3.get().getPort("S1");
	private static final Port csPort = LocalEV3.get().getPort("S3");

	//Setting up ultrasonic sensor
	public static UARTSensor usSensor = new EV3UltrasonicSensor(usPort);
	public static SampleProvider usValue = usSensor.getMode("Distance");

	//Setting up gyro sensor 
	public static EV3GyroSensor gyroSensor = new EV3GyroSensor(gyroPort);
	public static SampleProvider gyroValue = gyroSensor.getMode("Angle");
	
	//Setting up light sensor

	public static UARTSensor lsSensor = new EV3ColorSensor(lsPort);
	public static SampleProvider lsValue = lsSensor.getMode("Red");
	
	public static EV3ColorSensor csSensor = new EV3ColorSensor(csPort);
	public static SampleProvider csValue = csSensor.getRGBMode();

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 10.45;
	public static final double TILE_SIZE = 30.48;
	
	public static final int startOption = 0;
	public static final double[] startCorner = {2.0, 2.0};
	public static final double[] endCorner = {5.0, 5.0};
	public static final RingColors targetRing = RingColors.ORANGE;

	
	

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
		AngleSampler gyro = new AngleSampler();
		GyroPoller gyroPoller = new GyroPoller(gyroValue, gyro);
		
		
		ColorClassifier CSLocal = new ColorClassifier(odo, nav, targetRing);
		ColorPoller csPoller = new ColorPoller(csValue, CSLocal);
		double[] xyt;

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
		Thread usPollerThread = new Thread(usPoller);
		usPollerThread.start();
		Thread lsPollerThread = new Thread(lsPoller);
		lsPollerThread.start();
		
		try {
			usPollerThread.join();
			lsPollerThread.join();
			usSensor.close();
			lsSensor.close();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// set position according to startOption
		switch(startOption) {
		case 1:
			odo.setXYT(6*TILE_SIZE, 0, 270);
			break;
		case 3:
			odo.setXYT(0, 6*TILE_SIZE, 90);
			break;
		case 2:
			odo.setXYT(6*TILE_SIZE, 6*TILE_SIZE, 180);
			break;
		}
		xyt = odo.getXYT();
		
		nav.travelTo(xyt[0]/TILE_SIZE, startCorner[1] - 1);
		nav.travelTo(startCorner[0] - 1, startCorner[1] - 1);
		Thread navThread  = new Thread(nav);
		navThread.start();
		
		try {
			navThread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		// turns to 0, reset gyro to 0
		nav.turnTo(odo.getXYT()[2], 0);
		
		Thread gyroPollerThread = new Thread(gyroPoller);
		gyroPollerThread.start();
		
		gyroSensor.reset();
		nav.setGyro(gyro);
		
		
		// find ring part
		
		// spiral code
//		USDetector USDetect = new USDetector();
//		UltrasonicPoller usPoller2 = new UltrasonicPoller(usValue2, USDetect);
		
		
		
		initSpiral(nav, startCorner, endCorner);
		Thread csPollerThread = new Thread(csPoller);
		csPollerThread.start();
		navThread = new Thread(nav);
		navThread.start();
//		Thread usPollerThread2 = new Thread(usPoller2);
//		usPollerThread2.start();
//		
//		
		try {
			navThread.join();
			csPollerThread.join();
			csSensor.close();
//			usPollerThread2.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// found the ring
		// navigates to the end corner
		nav.setRunning(true);
		xyt =  odo.getXYT();
		if (xyt[2] >= 46 && xyt[2] <= 135) {
			nav.syncTravelTo(((int)xyt[0]/TILE_SIZE+0.5), xyt[1]/TILE_SIZE );	
		} else if (xyt[2] >= 136 && xyt[2] <= 225) {
			nav.syncTravelTo(xyt[0]/TILE_SIZE, ((int)xyt[1]/TILE_SIZE-0.5));	
		} else if (xyt[2] >= 226 && xyt[2] <= 315) {
			nav.syncTravelTo(((int)xyt[0]/TILE_SIZE-0.5), xyt[1]/TILE_SIZE );	
		} else {
			nav.syncTravelTo(xyt[0]/TILE_SIZE, ((int)xyt[1]/TILE_SIZE+0.5));
		}
		
		nav.syncTravelTo(endCorner[0]+0.5, odo.getXYT()[1]/TILE_SIZE);	
		nav.syncTravelTo(endCorner[0]+0.5, endCorner[1]);	
		nav.syncTravelTo(endCorner[0],  endCorner[1]);	

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

package ca.mcgill.ecse211.Lab5;

import java.text.DecimalFormat;

import ca.mcgill.ecse211.Color.ColorClassifier;
import ca.mcgill.ecse211.Color.ColorPoller;
import ca.mcgill.ecse211.Odometer.Odometer;
import ca.mcgill.ecse211.Odometer.OdometerExceptions;
import ca.mcgill.ecse211.Ultrasonic.USDetector;
import ca.mcgill.ecse211.Ultrasonic.USLocalizer;
import ca.mcgill.ecse211.Ultrasonic.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class Display extends Thread implements Runnable {

	  private Odometer odo;
	  private UltrasonicPoller usPoller;
	  private ColorClassifier color;
	  private ColorPoller csPoller;
	  private USDetector usDetector;
	  private TextLCD lcd;
	  private double[] position;
	  private final long DISPLAY_PERIOD = 25;
	  private long timeout = Long.MAX_VALUE;

	  /**
	   * This is the class constructor
	   * 
	   * @param odoData
	   * @throws OdometerExceptions 
	   */
	  public Display(TextLCD lcd) throws OdometerExceptions {
	    odo = Odometer.getOdometer();
	    this.lcd = lcd;
	  }

	  /**
	   * This is the overloaded class constructor
	   * 
	   * @param odoData
	   * @throws OdometerExceptions 
	   */
	  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
	    odo = Odometer.getOdometer();
	    this.timeout = timeout;
	    this.lcd = lcd;
	  }

	public void run() {
	    lcd.clear();
	    
	    long updateStart, updateEnd;

	    long tStart = System.currentTimeMillis();
	    do {
	      updateStart = System.currentTimeMillis();

	      // Retrieve x, y and Theta information
	      position = odo.getXYT();

	  	  if (ColorClassifier.detectedRing != null) {
	  		  lcd.clear();
	    	  	  lcd.drawString("Object Detected!", 0, 0);
		      lcd.drawString(ColorClassifier.detectedRing.toString(), 0, 1);
//		      	if (ColorClassifier.targetDetected) {
//		      		lcd.drawString("Target Detected!", 0, 2);
//		      	}
		      	try {
		      		Thread.sleep(1000);
		      		lcd.clear();
		      	} catch (InterruptedException e) {
				// TODO Auto-generated catch block
		      		e.printStackTrace();
			} 

	      }



	      if (odo != null) {
		      // Print x,y, and theta information
		      DecimalFormat numberFormat = new DecimalFormat("######0.00");
		      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
		      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
		      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
		      
		  	  if (ColorClassifier.detectedRing != null) {
		  		  lcd.clear();
		    	  	  lcd.drawString("Object Detected!", 0, 0);
			      lcd.drawString(ColorClassifier.detectedRing.toString(), 0, 1);
			      	try {
			      		Thread.sleep(1000);
			      		lcd.clear();
			      	} catch (InterruptedException e) {
					// TODO Auto-generated catch block
			      		e.printStackTrace();
				} 
			     
			    if (ColorClassifier.targetDetected) {
			    		lcd.clear();
			    		lcd.drawString("Target Information", 0, 0);
			    		lcd.drawString("X: " + ColorClassifier.targetLocation[0], 0, 1);
			    		lcd.drawString("Y: " + ColorClassifier.targetLocation[1], 0, 2);
			    		lcd.drawString("T: " + ColorClassifier.targetLocation[2], 0, 3);
			    		lcd.drawString("Red Sample: " + ColorClassifier.targetRGBValues[0], 0, 4);
			    		lcd.drawString("Green Sample: " + ColorClassifier.targetRGBValues[1], 0, 5);
			    		lcd.drawString("Blue Sample: " + ColorClassifier.targetRGBValues[2], 0, 6);
			    }

		      }
	      }
	      
	      

//	      
//	      if (usPoller != null) {
//	          lcd.drawString("Distance: " + numberFormat.format(usPoller.distance), 0, 3);
//	          //lcd.drawString("DMR: " + numberFormat.format(usPoller.getController().dotMagnitudeRatio), 0, 4);
//	      }
//	      
//	      lcd.drawString("Nav X: " + numberFormat.format(Navigation.destX), 0, 5);
//	      lcd.drawString("Nav Y: " + numberFormat.format(Navigation.destY), 0, 6);
//	      lcd.drawString("Nav T: " + numberFormat.format(Navigation.destT), 0, 7);
	      
//	      lcd.drawString("Theta A: " + numberFormat.format(USLocalizer.thetaA), 0, 5);
//	      lcd.drawString("Theta B: " + numberFormat.format(USLocalizer.thetaB), 0, 6);
//	      lcd.drawString("Theta Av: " + numberFormat.format(USLocalizer.thetaAv), 0, 7);
	      
//	      // DEBUG
//	      int[] tachos = odo.getTachoCount();
//	      lcd.drawString("TL: " + numberFormat.format(tachos[0]), 0, 3);
//	      lcd.drawString("TR: " + numberFormat.format(tachos[1]), 0, 4);
	      
//	      try {
//			if (OdometryCorrection.getInstance() != null) {
//				  OdometryCorrection OC = OdometryCorrection.getInstance();
//				  	lcd.drawString("LS: " + numberFormat.format(OC.lightSensorIntensity), 0, 5);
//				  	lcd.drawString("Std dev: " + numberFormat.format(OC.stdDev), 0, 6);
//				  	lcd.drawString("Mean: " + numberFormat.format(OC.mean), 0, 7);
//			  }
//		  } catch (OdometerExceptions e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//		  }
	      
	      
	      // this ensures that the data is updated only once every period
	      updateEnd = System.currentTimeMillis();
	      if (updateEnd - updateStart < DISPLAY_PERIOD) {
	        try {
	          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
	        } catch (InterruptedException e) {
	          e.printStackTrace();
	        }
	      }
	    } while ((updateEnd - tStart) <= timeout);

	  }

	}



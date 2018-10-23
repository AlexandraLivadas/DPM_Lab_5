package ca.mcgill.ecse211.Gyro;

import ca.mcgill.ecse211.Light.LightController;
import lejos.robotics.SampleProvider;

public class GyroPoller extends Thread {

	private SampleProvider gyro;
	private GyroController cont;
	private float[] gyroData;
	public float value;

	public volatile boolean running;

	public GyroPoller(SampleProvider gyro, GyroController cont) {
		this.gyro = gyro;
		this.gyroData = new float[gyro.sampleSize()];
		this.cont = cont;
		this.running = true;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert LS result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		while (cont.isRunning()) {
			
			if (cont.getLock() != null) {
				Object lock = cont.getLock();
				synchronized(lock) {
					try {
						lock.wait();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
			
			
			gyro.fetchSample(gyroData, 0); // acquire data
			value = (gyroData[0]); // extract from buffer, cast to int
			cont.process(value); // now take action depending on value
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}


}

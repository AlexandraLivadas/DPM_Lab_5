package ca.mcgill.ecse211.Color;

import lejos.robotics.SampleProvider;

public class ColorPoller extends Thread {
	private SampleProvider cs;
	private ColorController cont;
	private float[] csData;
	public double[] rgbValues;

	public volatile boolean running;

	public ColorPoller(SampleProvider cs, ColorController cont) {
		this.cs = cs;
		this.csData = new float[cs.sampleSize()];
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
			
			
			cs.fetchSample(csData, 0); // acquire data
			rgbValues[0] = (double)csData[0]; // extract from buffer, cast to int
			rgbValues[1] = (double)csData[1]; // extract from buffer, cast to int
			rgbValues[2] = (double)csData[2]; // extract from buffer, cast to int
			cont.process(rgbValues); // now take action depending on value
			try {
				Thread.sleep(15);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}

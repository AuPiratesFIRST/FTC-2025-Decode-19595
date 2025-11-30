package org.firstinspires.ftc.teamcode.Auto;

/**
 * Simple timer utility for autonomous routines.
 * Provides elapsed time tracking functionality.
 */
public class Timer {
    private long startTime;
    private boolean running;
    
    /**
     * Create a new timer (not started)
     */
    public Timer() {
        this.startTime = 0;
        this.running = false;
    }
    
    /**
     * Reset and start the timer
     */
    public void resetTimer() {
        this.startTime = System.currentTimeMillis();
        this.running = true;
    }
    
    /**
     * Get elapsed time in milliseconds
     * 
     * @return Elapsed time in milliseconds
     */
    public long getElapsedTimeMs() {
        if (!running) {
            return 0;
        }
        return System.currentTimeMillis() - startTime;
    }
    
    /**
     * Get elapsed time in seconds
     * 
     * @return Elapsed time in seconds
     */
    public double getElapsedTimeSeconds() {
        return getElapsedTimeMs() / 1000.0;
    }
    
    /**
     * Stop the timer
     */
    public void stop() {
        this.running = false;
    }
    
    /**
     * Check if timer is running
     * 
     * @return True if timer is running
     */
    public boolean isRunning() {
        return running;
    }
}


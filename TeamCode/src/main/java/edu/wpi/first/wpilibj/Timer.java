package edu.wpi.first.wpilibj;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for a time related items.
 */
public class Timer {
    private ElapsedTime time;
    private boolean running;
    private double lastTime;

    public Timer() {
        time = new ElapsedTime();
    }

    public double get() {
        if (running) {
            return time.seconds();
        } else {
            return lastTime;
        }
    }

    public void start() {
        running = true;
    }

    public void restart() {
        time.reset();
        running = true;
    }

    public void stop() {
        running = false;
        lastTime = time.seconds();
    }

    public boolean hasElapsed(double seconds) {
        return get() >= seconds;
    }

    public void reset() {
        time.reset();
    }
}
package frc.looper;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;

public class Looper {

  private double delayT = 0;
  private Notifier notifier;
  private boolean running = false;
  private ArrayList<Loop> loopArray;

  private double counter = 0;

  /**
   * The runnable looper
   */
  private Runnable masterLoop = () -> {
    if (running) {
      for (Loop l : loopArray) {
        l.runLoop();
      }
    }
    counter++;
    // System.out.println("RUNNING: " + running + " TIME/100: " + counter);

  };

  /**
   * Master loop class, handles all other loops
   * 
   * @param delayTime : time to wait in between cycles
   */
  public Looper(double delayTime) {
    delayT = delayTime;
    loopArray = new ArrayList<Loop>();
    notifier = new Notifier(masterLoop);
  }

  /**
   * add a new loop to the arraylist
   * 
   * @param loop : the loop method called with every iteration
   */
  public void register(Loop loop) {
    loopArray.add(loop);
  }

  /**
   * Starts all the loops
   */
  public void startLoops() {
    if (!running) {
      System.out.println("starting loops");
      running = true;
      notifier.startPeriodic(delayT);
    }
  }

  /**
   * Ends the loops
   */
  public void endLoops() {
    running = false;
    notifier.stop();
  }
}
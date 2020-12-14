package org.firstinspires.ftc.teamcode.Components.Software;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class HardwareThreadExecutor {

    ExecutorService hardwareThreadExecutor;
    ArrayList<Runnable> tasks;

    public HardwareThreadExecutor(ArrayList<Runnable> tasks) {
        hardwareThreadExecutor = Executors.newFixedThreadPool(tasks.size());
        this.tasks = tasks;
    }

    public void initiateExecutor() {
        for(Runnable task : tasks) {
            hardwareThreadExecutor.execute(task);
        }
    }

    public void shutdownExecutor() { hardwareThreadExecutor.shutdown(); }

}

package org.firstinspires.ftc.teamcode.teleOp;

import org.firstinspires.ftc.teamcode.freightFrenzy.AZUtil;

public abstract class FFCommandExecutor {

    boolean runInParallel = true;
    public FFCommandExecutor(boolean p){
        runInParallel = p;
    }

    public FFCommandExecutor(){
    }

    public FFCommandExecutor(String poolGroup){

    }

    boolean isRunning = false;

    public void run(){
        if(!isRunning){
            isRunning = true;
            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {
                    execute();
                    isRunning = false;
                }
            });
        }
    }
    abstract void execute();
}
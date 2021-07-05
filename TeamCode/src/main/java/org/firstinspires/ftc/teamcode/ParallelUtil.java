package org.firstinspires.ftc.teamcode;


public class ParallelUtil {

    // todo: write your code here
    
    public static void runInParallel(Runnable r){
            new Thread(r).start();
    }
    
    
    private void example(){
        ParallelUtil.runInParallel(
            new Runnable(){
                public void run(){
                    //Your statements here
                }
            }
            );
    }
}
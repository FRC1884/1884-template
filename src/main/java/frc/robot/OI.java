package frc.robot;


public class OI{
    private static OI instance;

    public static OI getInstance(){
        if(instance==null){
            instance = new OI();
        }
        return instance;
    }
}
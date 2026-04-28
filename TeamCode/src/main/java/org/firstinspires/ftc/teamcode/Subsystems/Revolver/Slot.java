package org.firstinspires.ftc.teamcode.Subsystems.Revolver;

public class Slot {
    double angle;
    boolean IsthereBall;
    public Slot(double angle){
        IsthereBall=false;
        this.angle =angle;
    }
    public double getAngle(){
        return angle;
    }
    public boolean IsthereBall(){
        return IsthereBall;
    }
    public void setIsthereBall(boolean lt){
        this.IsthereBall = lt;
    }

}

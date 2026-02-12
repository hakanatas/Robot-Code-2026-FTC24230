package org.firstinspires.ftc.teamcode.Subsystems.Tilter;

import static org.firstinspires.ftc.teamcode.Subsystems.Tilter.Tilter.tilterState.IDLE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.wrappers.WServo;
import org.firstinspires.ftc.teamcode.wrappers.WSubsystem;
@Config
public class Tilter  extends WSubsystem {
    public static double Tilterpos =0;
        WServo masterTilterMotor;
        WServo slaveTilterMotor;

        public enum tilterState{
            IDLE,
            DEPLOYED
        }
        static tilterState currentState= IDLE;

        @Override
        public void init(HardwareMap hardwareMap) {
            masterTilterMotor = new WServo(hardwareMap.get(Servo.class,"tilterMaster"));
            masterTilterMotor.setDirection(Servo.Direction.REVERSE);
            slaveTilterMotor = new WServo(hardwareMap.get(Servo.class,"tilterSlave"));
            slaveTilterMotor.setDirection(Servo.Direction.FORWARD);
        }

        @Override
        public void periodic() {
            switch (currentState){
                case IDLE:
                    masterTilterMotor.setPosition(0);
                    slaveTilterMotor.setPosition(0);
                    break;
                case DEPLOYED:
                    masterTilterMotor.setPosition(0.5);
                    slaveTilterMotor.setPosition(0.5);
                    break;
            }
        }
        public void setState(tilterState st){
            if(currentState!=st){
                currentState=st;
            }
        }

        @Override
        public void read() {
        }

        @Override
        public void write() {
        }

        @Override
        public void reset() {
        }
    }

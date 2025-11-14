package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

//low-key don't need ts
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turntable;
import frc.robot.subsystems.Turntable.TurntableStates;

import edu.wpi.first.wpilibj.Servo;


public class Dispenser extends SubsystemBase
{
    public enum DispenserStates
    {
        StateDispense,
        StateReload,
        StateClosed
    }

    int timer;

    public DispenserStates currentState = DispenserStates.StateClosed;
    public DispenserStates requestedState = DispenserStates.StateClosed;
    
    private final Servo topMotor = new Servo(0);
    private final Servo bottomMotor = new Servo(1);
    private double topPos = 0;
    private double bottomPos = 0;
    //private final Turntable m_Turntable = new Turntable(() -> {m_driverController.getLeftX()} ); //fix

    public Turntable m_Turntable;
    public Thingamabob referenceManager;
   

    public Dispenser(Turntable m_Turntable, Thingamabob referenceManager){
       topMotor.setAngle(0);
       bottomMotor.setAngle(0);
       this.m_Turntable = m_Turntable;
       this.referenceManager = referenceManager;
    }

    public void periodic(){
        switch(requestedState){
            case StateClosed:
                topPos = 0;
                bottomPos = 0;
                referenceManager.requestedState = Thingamabob.ThingamaStates.StateBusyDoingSomething;
                break;
            case StateDispense:
                topPos = 0; 
                bottomPos = 90;
                timer++;
                if (timer >= 30){
                    requestedState = DispenserStates.StateReload;
                    timer = 0;
                }
                break;    
            case StateReload:
                if(bottomMotor.getPosition()==0)
                {
                    topPos = 90;
                }
                else
                {
                    bottomPos = 0;
                }
                timer++;
                if(timer >= 30) {
                    requestedState = DispenserStates.StateClosed;
                    timer = 0;
                }
                break;
            }
      }
    //@Override
    public void runControlLoop()
    {
        if(referenceManager.isReloadReady){
            topMotor.setAngle(topPos);
            bottomMotor.setAngle(bottomPos);
        }
    }
   
 
    public void requestState(DispenserStates rState) {
        requestedState = rState;
    }
}
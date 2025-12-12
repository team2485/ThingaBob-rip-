package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turntable.TurntableStates;

public class Thingamabob extends SubsystemBase {

    public enum ThingamaStates{
        StateBusyDoingSomething,
        StateReloading,
    }

    public ThingamaStates currentState = ThingamaStates.StateBusyDoingSomething;
    public ThingamaStates requestedState = ThingamaStates.StateBusyDoingSomething;

    public Turntable referenceTurntable;
    public Arm referenceArm;
    public Dispenser referenceDispenser;
    public boolean isReloadReady;

    public Thingamabob(Turntable referenceTurntable, Arm referenceArm, Dispenser referenceDispenser){
        this.referenceTurntable = referenceTurntable;
        this.referenceArm = referenceArm;
        this.referenceDispenser = referenceDispenser;
        isReloadReady = false;
    }
    
    
    @Override
    public void periodic(){
        switch(currentState){
            case StateBusyDoingSomething:
                isReloadReady = false;
                break;
            case StateReloading:
                if (Math.abs(referenceTurntable.position) >= 0.1) {
                    referenceArm.requestedState = Arm.ArmStates.StateZero;
                    if (referenceArm.currentState == Arm.ArmStates.StateZero) {
                        referenceTurntable.requestedState = Turntable.TurntableStates.StateZero;
                        isReloadReady = true;
                        if(referenceTurntable.currentState == Turntable.TurntableStates.StateZero){
                            referenceDispenser.requestedState = Dispenser.DispenserStates.StateDispense;
                            //System.out.println("Works Turntable");
                            if(referenceDispenser.currentState == Dispenser.DispenserStates.StateDispense){
                                referenceDispenser.requestedState = Dispenser.DispenserStates.StateReload;
                                //System.out.println("Breaking");
                                requestedState = ThingamaStates.StateBusyDoingSomething;
                                break;
                            }
                        }
                    }
                }
                
                else{
                    referenceTurntable.requestedState = Turntable.TurntableStates.StateSide;
                }
                break;
        }

        currentState = requestedState;
    }

    public void requestState(ThingamaStates req){

        requestedState = req;

    }

}
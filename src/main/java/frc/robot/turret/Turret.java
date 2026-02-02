package frc.robot.turret;

import com.ctre.phoenix6.hardware;
/* 
 * Outlining the goals -> 
 * there are currently two mechanical setbacks that need to be fixed in code, telling the rotation of the turret and it's "zone" which is done by a 
 * CANcoder (WCP Hex CANcoder) 
 * and the ability for the turret to move more than 360 degrees with the proper fault protection from the motors. 
 * Furthermore Motion magic would be used in replacement to PID due to the instability of PID and the time constraint of the pre-coding the robot
 *
 * The function of this class, to counter both mechanical setbacks, allow turret autotracking with the help or the AUX camera from photon vision etc.
 * 
 * There exists a "deadzone" for the climber
 * you have a "reset heading" position to which you cannot overextend or the wires snap
 * It's a kraken motor geared down to 12:1 connected to a larger cog that is x amount
 *
 */
import com.ctre.phoenix6.hardware.TalonFX;

public class Turret {

    public Turret() {
        TalonFX m_turretMotor = new TalonFX(TurretConst.m_motorID);

    }
}

package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShooterState {
        kStopped,
        kSpinning,
        kShooting,
        kPreAmp,
        kAmp,
        kPostAmp
    }

    private CANSparkFlex m_motor1;
    private CANSparkFlex m_motor2;
    private CANSparkMax m_ampArmMotor;
    private ShooterState m_state = ShooterState.kStopped;
    private double m_speed = ShooterConstants.kShootSpeed;

    public ShooterSubsystem() {
        m_motor1 = new CANSparkFlex(ShooterConstants.kRightShooterMotorCANId, MotorType.kBrushless);
        m_motor2 = new CANSparkFlex(ShooterConstants.kLeftShooterMotorCANId, MotorType.kBrushless);
        m_ampArmMotor = new CANSparkMax(ShooterConstants.kAmpShooterMotorCANID, MotorType.kBrushless);

        m_motor1.restoreFactoryDefaults();
        m_motor2.restoreFactoryDefaults();

        // set coast mode to prevent damage
        m_motor1.setIdleMode(IdleMode.kBrake);
        m_motor2.setIdleMode(IdleMode.kBrake);
    }

    public double getAmpSpeed() {
        return m_ampArmMotor.get();
    }

    /**
     * set the state of the shooter
     * @param state the new state to set
     */
    public void setState(ShooterState state) {
        m_state = state;

        double armSpeed = switch(m_state) {
            case kPreAmp -> -1;
            case kAmp -> -0.2;
            case kPostAmp -> 0.5; // increase post amp speed
            default -> 0;
        };

        m_ampArmMotor.set(armSpeed);

        setSpeed(m_speed);
    }

    public void setSpeed(double speed) {
        m_speed = speed;

        double actualSpeed = switch(m_state) {
            case kStopped -> 0;
            case kSpinning, kShooting -> m_speed;
            case kAmp, kPreAmp, kPostAmp  -> ShooterConstants.kAmpShotSpeed;
        };

        m_motor1.set(actualSpeed);
        m_motor2.set(actualSpeed);
    }

    /**
     * get the current state of the shooter
     * @return the state of the shooter
     */
    public ShooterState getState() {
        return m_state;
    }
}

package frc.robot.subsystems.note_handling;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.UTBIntakerConstants;

public class UTBIntakerSubsystem extends Intaker {
    
    private CANSparkMax m_intakeMotor1;
    private CANSparkMax m_intakeMotor2;

    private IntakerMotorState m_state = IntakerMotorState.kStopped;

    public UTBIntakerSubsystem() {
        m_intakeMotor1 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor1CANId, MotorType.kBrushless);
        m_intakeMotor2 = new CANSparkMax(UTBIntakerConstants.kIntakeMotor2CANId, MotorType.kBrushless);
    }

    @Override
    public IntakerPosition getIntakerPosition() {
        // intaker is always down
        return IntakerPosition.kDown;
    }

    @Override
    public IntakerMotorState getMotorState() {
        return m_state;
    }

    public void setMotorState(IntakerMotorState state) {
        m_state = state;

        double speed = switch (m_state) {
            case kReversed -> UTBIntakerConstants.kReverseMotorSpeed;
            case kStopped -> 0;
            case kIntaking -> UTBIntakerConstants.kIntakeMotorSpeed;
        };

        m_intakeMotor1.set(speed);
        m_intakeMotor2.set(speed);
    }

}

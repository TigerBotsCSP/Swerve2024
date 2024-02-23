package frc.robot.subsystems.attachment;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    public enum PivotPosition {
        kIntakePosition(PivotConstants.kIntakePos, PivotConstants.kIntakePos),
        kSpeakerPosition(PivotConstants.kSpeakerMin, PivotConstants.kSpeakerMax),
        kAmpPosition(PivotConstants.kAmpPos, PivotConstants.kAmpPos);

        public double lowLimit, highLimit;

        PivotPosition(double low, double high) {
            lowLimit = Math.max(PivotConstants.kGlobalMin, low);
            highLimit = Math.min(PivotConstants.kGlobalMax, high);
        }
    }

    private final CANSparkMax m_leftPivotMotor;
    private final SparkPIDController m_pivotPID;
    private final RelativeEncoder m_pivotEncoder;

    private PivotPosition m_pivotPosition = PivotPosition.kSpeakerPosition;
    private double m_pivotSetpoint = PivotConstants.kIntakePos;

    public PivotSubsystem() {
        m_leftPivotMotor = new CANSparkMax(PivotConstants.kLeftPivotMotorCANId, MotorType.kBrushless);

        m_leftPivotMotor.restoreFactoryDefaults();
        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);

        m_pivotPID = m_leftPivotMotor.getPIDController();

        // set false to not pop off chain by going wrong way
        m_pivotPID.setPositionPIDWrappingEnabled(false);

        m_pivotEncoder = m_leftPivotMotor.getEncoder();
        m_pivotPID.setFeedbackDevice(m_pivotEncoder);

        // set pivot PID coefficients
        m_pivotPID.setP(PivotConstants.PivotPID.kP);
        m_pivotPID.setI(PivotConstants.PivotPID.kI);
        m_pivotPID.setD(PivotConstants.PivotPID.kD);
        m_pivotPID.setIZone(PivotConstants.PivotPID.kIz);
        m_pivotPID.setFF(PivotConstants.PivotPID.kFF);
        m_pivotPID.setOutputRange(PivotConstants.PivotPID.kMinOutput, PivotConstants.PivotPID.kMaxOutput);
    }

    @Override
    public void periodic() {
        m_pivotPID.setReference(m_pivotSetpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("pivot setpoint", m_pivotSetpoint);
        SmartDashboard.putNumber("pivot position", m_pivotEncoder.getPosition());
    }

    /**
     * Set the general mode of the shooter.
     * This specifies the available ranges for the pivot.
     * @param pos The new general position for the shooter
     */
    public void setPosition(PivotPosition pos) {
        m_pivotPosition = pos;
        setPrecisePosition(m_pivotSetpoint);
    }

    /**
     * Set the specific setpoint for the feeder
     * 
     * @param setpoint the new setpoint for the shooter
     */
    public void setPrecisePosition(double setpoint) {
        if (Double.isNaN(setpoint)) {
            System.out.println("Got NAN pivot setpoint");
            return;
        }

        m_pivotSetpoint = Math.max(m_pivotPosition.lowLimit, Math.min(setpoint, m_pivotPosition.highLimit));
    }

    public double getPrecisePosition() {
        return m_pivotSetpoint;
    }

    public boolean isPivotReady() {
        return Math.abs(m_pivotSetpoint - m_pivotEncoder.getPosition()) < PivotConstants.kPositionDeadzone;
    }
}

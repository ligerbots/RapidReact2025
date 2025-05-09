package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.ShooterSpeeds;
// import frc.robot.subsystems.Vision.VisionMode;

public class ShooterCommand extends Command {
    /**
     * Creates a new ShooterCommand.
     */
    Shooter m_shooter;
    Intake m_intake;
    // Vision m_vision;
    boolean m_upperHub;
    double m_distance;
    boolean m_useVision;

    LigerTimer m_shootDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME);
    LigerTimer m_intakeDelay = new LigerTimer(Constants.SHOOTER_INTAKE_WAIT_TIME);
    LigerTimer m_shootBall1Time = new LigerTimer(Constants.SHOOT_BALL1_WAIT_TIME);
    LigerTimer m_shootBall2Time = new LigerTimer(Constants.SHOOT_BALL2_WAIT_TIME);
    LigerTimer m_visionTime = new LigerTimer(2.0); // give the vision at most 2 seconds to find the target

    ShooterSpeeds m_shooterSpeeds;
    
    public static final double DEFAULT_DISTANCE_TO_THE_HUB = 9.0 * 12.0; // 9 feet

    enum State {
        // FINDING_VISION_TARGET, 
        SPEED_UP_SHOOTER, WAIT_FOR_SHOOTER, TURN_ON_CHUTE, TURN_ON_INTAKE, 
        WAIT_FOR_SHOOT_BALL1, WAIT_FOR_SHOOT_BALL2;
    }

    State m_state;

    // public ShooterCommand(Shooter shooter, Intake intake, Vision vision, boolean upperHub) {
    //     m_shooter = shooter;
    //     m_intake = intake;
    //     m_vision = vision;
    //     m_upperHub = upperHub;
    //     m_useVision = true;
    // }

    public ShooterCommand(Shooter shooter, Intake intake, double distance, boolean upperHub) {
        m_shooter = shooter;
        m_intake = intake;
        m_distance = distance;
        m_upperHub = upperHub;
        m_useVision = false;

        SmartDashboard.putString("shooter/cmdState", "INIT");

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if (m_upperHub == false || ! m_useVision) { // if lowerhub or if distance pre-defined, skip vision
            m_state = State.SPEED_UP_SHOOTER;
        // } else {
        //     // turn on vision finding, just in case, and it does not hurt if already done
        //     m_vision.setMode(VisionMode.HUBFINDER);
        //     m_visionTime.start();
        //     m_state = State.FINDING_VISION_TARGET;
        // }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_state) {
            // case FINDING_VISION_TARGET:
            //     m_distance = m_vision.getDistance();
            //     // go to the next state once the target is found
            //     if (m_distance > 1.0)
            //         m_state = State.SPEED_UP_SHOOTER;
            //     else if (m_visionTime.hasElapsed()) {
            //         m_state = State.SPEED_UP_SHOOTER;
            //         // if still can't find the target, just use 9ft as the distance
            //         m_distance = DEFAULT_DISTANCE_TO_THE_HUB;
            //     }
            //     else 
            //         break;
            //     // allows fall through to the next state if found the target

            case SPEED_UP_SHOOTER:
                m_shooterSpeeds = Shooter.calculateShooterSpeeds(m_distance, m_upperHub);
                // System.out.println("distance " + m_distance + " chute speed " + m_shooterSpeeds.chute + " bottom " + m_shooterSpeeds.bottom 
                //         + " top " + m_shooterSpeeds.top);

                // turn on the two motors on the shooter, let the chute and intake wait for the
                // shots
                // m_shooter.setShooterSpeeds(0.3, 0.3);
                m_shooter.setShooterRpms(m_shooterSpeeds.top, m_shooterSpeeds.bottom);
                m_state = State.WAIT_FOR_SHOOTER;
                m_shootDelay.start();
                break;

            case WAIT_FOR_SHOOTER:
                if (m_shootDelay.hasElapsed())
                    m_state = State.TURN_ON_CHUTE;
                else {
                    // fall through if timer elapsed
                    break;
                }

            case TURN_ON_CHUTE:
                // turn on the chute once the shooter is ready
                System.out.println("Turn on chute");

                m_shooter.setChuteSpeed(m_shooterSpeeds.chute);
                m_state = State.WAIT_FOR_SHOOT_BALL1;
                m_shootBall1Time.start();
                break;
                
            case WAIT_FOR_SHOOT_BALL1:
                if (m_shootBall1Time.hasElapsed())
                    m_state = State.TURN_ON_INTAKE;
                else {
                    // allow fall through if elapse
                    break;
                }
            
            case TURN_ON_INTAKE:
                System.out.println("Turn on intake");
                m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
                m_shootBall2Time.start();
                m_state = State.WAIT_FOR_SHOOT_BALL2;
                break;

            case WAIT_FOR_SHOOT_BALL2:
                //State.WAIT_FOR_SHOOT_BALL2 checked in isFinished method
                break;
        }

        SmartDashboard.putString("shooter/cmdState", m_state.toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setShooterSpeeds(0.0, 0.0);
        m_shooter.setChuteSpeed(0.0);
        m_intake.run(0.0);
        // if (m_vision != null) m_vision.setMode(Vision.DEFAULT_MODE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.WAIT_FOR_SHOOT_BALL2 && m_shootBall2Time.hasElapsed();
    }
}

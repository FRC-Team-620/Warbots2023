package frc.robot.Util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.FiringPins;
import frc.robot.Util.LEDs.LEDSubsystem.LEDAnimation;
import frc.robot.Util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;

    protected LEDAnimation noBallsAnim = LEDManager.STRIP0.fadeAnimation(1,
        30, 
        Color.kYellow,
        Color.kWhite
    );
    protected LEDAnimation oneBallAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kRed,
        Color.kOrangeRed,
        Color.kOrange
    );
    protected LEDAnimation twoBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kBlue,
        Color.kBlueViolet,
        Color.kPurple
    );

    // protected LEDAnimation noBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
    //     Color.kRed,
    //     Color.kOrangeRed,
    //     Color.kOrange
    // );
    // protected LEDAnimation twoBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
    //     Color.kBlue,
    //     Color.kBlueViolet,
    //     Color.kPurple
    // );
    // protected LEDAnimation oneBallAnim = LEDAnimation.transposeBlinking(0.04, 
    //     twoBallsAnim, 
    //     noBallsAnim
    // );

    public LEDIdleCommand(LEDSubsystem ledSubsystem, Intake intake, FiringPins firingPins) {
        this.intake = intake;
        this.firingPins = firingPins;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        if(this.firingPins.hasColor()) { 
            if(this.intake.getIntakeSwitch()) { // TWO balls
                this.twoBallsAnim.step();
            } else { // ONE ball
                this.oneBallAnim.step();
            }
        } else { // NO balls
            this.noBallsAnim.step();
        }
    }
}
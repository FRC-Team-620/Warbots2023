package frc.robot.util.LEDs;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    public static enum LEDManager {

        STRIP0(65, 9); // 65 lights, PWM port 9

        private LEDStrip strip;
        private LEDManager(int LEDCount, int PWMPort) {
            this.strip = new LEDStrip(LEDCount, PWMPort);
        }

        public LEDAnimation colorBlocksAnimation(double speed, int[] lengthPattern, Color... colors) {
            return this.strip.colorBlockAnimation(speed, lengthPattern, colors);
        }
        public LEDAnimation gradientAnimation(double speed, Color... colors) {
            return this.strip.gradientAnimation(speed, colors);
        }
        public LEDAnimation blinkingAnimation(double speed, Color... colors) {
            return this.strip.blinkingAnimation(speed, colors);
        }
        public LEDAnimation solidColorAnimation(Color color) {
            return this.strip.solidColorAnimation(color);
        }
        public LEDAnimation fadeTwoAnimation(double speed, int steps, boolean bidirectional, Color c1, Color c2) {
            return this.strip.fadeTwoAnimation(speed, steps, bidirectional, c1, c2);
        }
        public LEDAnimation fadeTwoAnimation(double speed, int steps, Color c1, Color c2) {
            return this.strip.fadeTwoAnimation(speed, steps, c1, c2);
        }
        public LEDAnimation fadeAnimation(double speed, int stepsPer, Color... colors) {
            return this.strip.fadeAnimation(speed, stepsPer, colors);
        }

        public void set(Color c) {
            this.strip.setSolidColor(c);
        }
        public void setColorBlocks(int[] lengths, Color... colors) {
            this.strip.setColorBlocks(lengths, colors);
        }
        public void setColorBlocks(int offset, int[] lengths, Color... colors) {
            this.strip.setColorBlocks(offset, lengths, colors);
        }
        public void setGradient(Color... colors) {
            this.strip.setGradient(colors);
        }
        public void setGradient(int offset, Color... colors) {
            this.strip.setGradient(offset, colors);
        }
        
    }

    public static class LEDAnimation {
        private double accumulation, speed;
        private Consumer<Integer> animation;
        public LEDAnimation(double speed, Consumer<Integer> animation) {
            this.accumulation = 0.0;
            this.speed = speed;
            this.animation = animation;
        }
        public void step() {
            this.animation.accept((int)this.accumulation);
            this.accumulation += this.speed;
        }
        public static LEDAnimation concatenate(double speed, LEDAnimation... animations) {
            return new LEDAnimation(speed, n -> animations[n%animations.length].step());
        }
    }

    private static class LEDStrip {

        private int PWMPort;
        private AddressableLED lights;
        private AddressableLEDBuffer buffer;
    
        public LEDStrip(int numLights, int PWMPort) {
            this.PWMPort = PWMPort;
            // Must be a PWM header
            this.lights = new AddressableLED(this.PWMPort);
    
            // Reuse buffer
            // Length is expensive to set, so only set it once, then just update data
            this.buffer = new AddressableLEDBuffer(numLights);
            this.lights.setLength(this.buffer.getLength());
    
            // Set the data
            this.lights.setData(this.buffer);
            this.lights.start();
        }

        public LEDAnimation gradientAnimation(double speed, Color... colors) {
            return new LEDAnimation(speed, n -> this.setGradient(n, colors));
        }

        public LEDAnimation colorBlockAnimation(double speed, int[] lengthPattern, Color... colors) {
            return new LEDAnimation(speed, n -> this.setColorBlocks(n, lengthPattern, colors));
        }

        public LEDAnimation blinkingAnimation(double speed, Color... colors) {
            return new LEDAnimation(speed, n -> this.setSolidColor(colors[n%colors.length]));
        }

        public LEDAnimation solidColorAnimation(Color color) {
            return new LEDAnimation(0, n -> this.setSolidColor(color));
        }

        public LEDAnimation fadeTwoAnimation(double speed, int steps, boolean bidirectional, Color c1, Color c2) {
            return new LEDAnimation(speed, n -> {
                n %= (bidirectional ? 2*steps : steps);
                int i = bidirectional ? -Math.abs(n-steps)+steps : n;
                double proportion = (double)i/steps;
                this.setSolidColor(new Color(
                    proportion*(c2.red-c1.red)+c1.red,
                    proportion*(c2.green-c1.green)+c1.green,
                    proportion*(c2.blue-c1.blue)+c1.blue
                ));
            });
        }

        public LEDAnimation fadeTwoAnimation(double speed, int steps, Color c1, Color c2) {
            return this.fadeTwoAnimation(speed, steps, true, c1, c2);
        }

        public LEDAnimation fadeAnimation(double speed, int stepsPer, Color... colors) {
            // Edge cases
            if(colors.length <= 0) return null;
            else if(colors.length == 1) 
                return this.solidColorAnimation(colors[0]);
            else if(colors.length == 2) 
                return this.fadeTwoAnimation(speed, stepsPer, true, colors[0], colors[1]);
            // Concatenating a series of fade animations
            LEDAnimation[] fades = new LEDAnimation[colors.length];
            for(int i = 0; i < colors.length; i++)
                fades[i] = this.fadeTwoAnimation(speed, stepsPer, false, colors[i], colors[(i+1)%colors.length]);
            return LEDAnimation.concatenate(speed/stepsPer, fades);
        }
    
        // returns an RGB representation of the light at a given index of the 'AddressableLEDBuffer'
        public Color colorAt(int idx) {
            return this.buffer.getLED(idx);
        }
    
        // sets the entire 'LEDStrip' to a given color
        public void setSolidColor(Color c) {
            for (int i = 0; i < this.buffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                this.buffer.setLED(i, c);
            }
            this.lights.setData(this.buffer);
        }
    
        // sets the 'LEDStrip' to a gradient melding between the given colors
        public void setGradient(Color... colors) {
            this.setGradient(0, colors);
        }
    
        // sets the 'LEDStrip' to a gradient melding between the given colors
        // 'offset' refers to the offset from the start that the gradient begins at, 
        // though it wraps around, so there will be no dead space.
        // 'offset' is useful for looping colors (continuously update 'offset')
        public void setGradient(int offset, Color... colors) {
            Color grad[];
            int lightsPerGrad = this.buffer.getLength()/colors.length;
            int finalGradPos = lightsPerGrad*colors.length-1;
            for(int i = 0; i < colors.length; i++) {
                grad = LEDStrip.colorGradient(colors[i], 
                    colors[(i+1)%colors.length], lightsPerGrad);
                for(int k = 0; k < lightsPerGrad; k++) {
                    int lightIdx = i*lightsPerGrad+k;
                    this.buffer.setLED((lightIdx+offset)%(finalGradPos+1), grad[k]);
                }
            }
            // correcting error resulting from integer division
            Color finalColor = this.colorAt(finalGradPos);
            for(int i = finalGradPos; i < this.buffer.getLength(); i++)
                this.buffer.setLED(i, finalColor);
            // setting the data
            this.lights.setData(this.buffer);
        }
    
        public void setColorBlocks(int[] lengths, Color... colors) {
            this.setColorBlocks(0, lengths, colors);
        }
    
        public void setColorBlocks(int offset, int[] lengths, Color... colors) {
            int idx = 0, bufferSize = this.buffer.getLength();
            for(int i = 0; idx < bufferSize; i++) {
                for(int k = 0; k < lengths[i%lengths.length] && idx+k < bufferSize; k++)
                    this.buffer.setLED((idx+k+offset)%bufferSize, colors[i%colors.length]);
                idx += lengths[i%lengths.length];
            }
            this.lights.setData(this.buffer);        
        }
    
        // returns an array of RGB color arrays encoding the gradient between two given
        // colors over a given number of 'steps' (the smoothness of the gradient)
        private static Color[] colorGradient(Color startColor, Color endColor, int steps) {
            Color gradient[] = new Color[steps], color;
            double proportion;
            for(int i = 0; i < gradient.length; i++) {
                proportion = (double)i/(gradient.length-1);
                color = new Color(
                    proportion*(endColor.red-startColor.red)+startColor.red,
                    proportion*(endColor.green-startColor.green)+startColor.green,
                    proportion*(endColor.blue-startColor.blue)+startColor.blue
                );
                gradient[i] = color;
            }
            return gradient;
        }
    }
    // Dummy constructor -- necessary in order to create commands around the LEDs and
    // to ensure that multiple commands are not run at once that set the LEDs (addRequirements)
    public LEDSubsystem() { }
}

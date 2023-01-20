package frc.robot;

public class DiminishingAverageHandler {
    
    private java.lang.Double average = null; // for nullability
    private final double previousWeight;

    /**
     * Handles running averages where past data should be expelled over time.
     * As new data comes in, the previous data is reduced in weight by some factor, 
     * and over time, this makes old data approach irrelevancy.
     * 
     * @param previousWeight The factor that previous data is multiplied by. This 
     * determines how important past data is. The larger, the more important. The closer
     * to zero, the less important.
     */
    public DiminishingAverageHandler(double previousWeight) {
        this.previousWeight = previousWeight;
    }

    /**
     * Getter for the current average
     * 
     * @return The current average
     */
    public double get() {
        return this.average;
    }

    /**
     * Give the DiminishingAverageHandler new data and return the new average.
     * If this is the first datum, then this will be returned as is.
     * 
     * @param data The new data
     * @return The new average
     */
    public double feed(double data) {
        if(average == null) {
            this.average = data;
            return this.average;
        }
        this.average = (this.average * this.previousWeight + data) / (1.0 + this.previousWeight);
        return average;
    }

    /**
     * Resets the DimishingAverageHandler, deleting past data
     */
    public void reset() {
        this.average = null;
    }

}

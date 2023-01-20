package frc.robot;

import java.lang.Double;

public class DiminishingAverageHandler {
    
    private Double average = null;
    private double previousWeight;

    public DiminishingAverageHandler(double previousWeight) {
        this.previousWeight = previousWeight;
    }

    public double get() {
        return this.average;
    }

    public double feed(double data) {
        if(average == null) {
            this.average = data;
            return this.average;
        }
        this.average = (this.average * this.previousWeight + data) / (1.0 + this.previousWeight);
        return average;
    }

    public void reset() {
        this.average = null;
    }

}

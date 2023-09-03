package org.texastorque.toast.lib;

public class DetectedObject {
    private String objectName;
    private double centerX, confidence;

    public DetectedObject(final String objectName, final double centerX, final double confidence) {
        this.objectName = objectName;
        this.centerX = centerX;
        this.confidence = confidence;
    }

    public String getObjectName() {
        return objectName;
    }

    public double getCenterX() {
        return centerX;
    }

    public double getConfidence() {
        return confidence;
    }
}

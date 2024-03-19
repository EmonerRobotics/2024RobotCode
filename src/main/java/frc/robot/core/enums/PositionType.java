package frc.robot.core.enums;

public enum PositionType {
    TARGET(-1, 0.55, 0.4),
    AMPHI(88, 0.55, 0.4),
    AUTO(14.5, 0.5, 0.4),
    GROUND(1.5, 1, 0.4);

    public final double positionDegree;
    public final double speedMultiplier;
    public final double threasold;

    private PositionType(
            double positionDegree,
            double speedMultiplier,
            double threshold
    ) {
        this.positionDegree = positionDegree;
        this.speedMultiplier = speedMultiplier;
        this.threasold = threshold;
    }
}

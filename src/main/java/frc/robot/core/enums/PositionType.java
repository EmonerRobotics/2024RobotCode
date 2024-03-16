package frc.robot.core.enums;

public enum PositionType {
    TARGET(-1, 1, 1.5),
    AMPHI(45, 0.5, 1),
    AUTO(14.5, 0.5, 1.5),
    GROUND(1.5, 1, 1.5);

    public final double positionDegree;
    public final double speedMultiplier;
    public final double threasold;

    private PositionType(
            double positionDegree,
            double speedMultiplier,
            double threasold
    ) {
        this.positionDegree = positionDegree;
        this.speedMultiplier = speedMultiplier;
        this.threasold = threasold;
    }
}

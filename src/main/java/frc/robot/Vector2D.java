package frc.robot;



public class Vector2D {

    private double x;
    private double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D add(Vector2D b) {
        return new Vector2D(x+b.x, y+b.y);
    }

    public Vector2D sub(Vector2D b) {
        return new Vector2D(x-b.x, y-b.y);
    }

    public Vector2D neg() {
        return new Vector2D(-x, -y);
    }

    public Vector2D mul(double b) {
        return new Vector2D(b*x, b*y);
    }

    public double dot(Vector2D b) {
        return x*b.x + y*b.y;
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public double lenth() {
        return Math.sqrt(this.dot(this));
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public static Vector2D iHat() {
        return new Vector2D(1.0, 0.0);
    }

    public static Vector2D jHat() {
        return new Vector2D(0.0, 1.0);
    }

}
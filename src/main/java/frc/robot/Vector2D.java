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

    public double dot(Vector2D b) {
        return x*b.x + y*b.y;
    }

    public double angle() {
        return Math.atan2(y, x);
    }




}
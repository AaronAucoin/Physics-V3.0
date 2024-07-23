package com.example.flatphysics;

public class Vector {
    private float x;
    private float y;

    public static Vector zeroVector = new Vector(0, 0);

    public Vector(float x, float y){
        this.x = x;
        this.y = y;
    }
    public float getX(){
        return x;
    }

    public float getY() {
        return y;
    }

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }

    public Vector multiply(float u){
        return new Vector(x * u, y * u);
    }

    public Vector divide(float u){
        float k = 1/u;
        return this.multiply(k);
    }

    public Vector addition(Vector u){
        return new Vector(x + u.x, y + u.y);
    }

    public Vector subtraction(Vector u){
        return new Vector(this.x = u.x, this.y - u.y);
    }

    public float dot(Vector other){
        return this.x * other.x + this.y * other.y;
    }

    public String toString(){
        return "Vector:(" + x + ", " + y + ")";
    }

    public Vector normalize() {
        float magnitude = this.getMagnitude();
        Vector normalized = this.divide(magnitude);
        return normalized;
    }

    public float getMagnitude(){
        return (float)Math.sqrt(x*x + y*y);
    }
}

package com.example.flatphysics;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;

import java.lang.reflect.Array;
import java.lang.reflect.WildcardType;
import java.util.ArrayList;

public class Ball {
    private final Vector G = new Vector(0, 9.8f);
    public static float dt = (float) 1/60;
    private Vector pos;
    private float r;
    public Vector velocity;
    private Vector acceleration = new Vector(0, 0);
    private float mass;
    private float inverseMass = (mass != 0) ? 1.0f / mass : 0.0f;
    private float density;
    private final Color color;
    final static Vector center = new Vector(flatPhysics.HEIGHT/2, flatPhysics.WIDTH/2);
    private final float restitution = 1;

    public Ball(float x, float y, float r, Color color) {
        this.pos = new Vector(x, y);
        this.r = r;
        this.color = color;
        this.velocity = new Vector(0,0);
        this.acceleration = new Vector(0,0);
        this.mass = r * 10;
    }

    public void setAcceleration(Vector acceleration){
        this.acceleration = acceleration;
    }
    public void setPos(Vector u){
        pos.setX(u.getX());
        pos.setY(u.getY());
    }

    public void addAcceleration(Vector newAcceleration){
        acceleration.addition(newAcceleration);
    }

    public Vector getAcceleration(){
        return acceleration;
    }

    public void setVelocity(Vector velocity) {
        this.velocity = velocity;
    }

    public float getX() {
        return pos.getX();
    }

    public float getY() {
        return pos.getY();
    }
    public float getr() {return r;}

    public Vector getPos(){
        return pos;
    }
    public float getRadi(){return r;}


    public void drawBall(GraphicsContext gc) {
        gc.setStroke(color);
        gc.setLineWidth(3);
        gc.strokeOval(pos.getX() - r, pos.getY() - r, 2 * r, 2 * r);
    }


    public boolean contains(double mouseX, double mouseY) {
        double dx = mouseX - (pos.getX());
        double dy = mouseY - (pos.getY());
        double distanceSquared = dx * dx + dy * dy;
        return distanceSquared <= r * r;
    }

    public void WallCollision() {
        if(pos.getX() <= 0 + r){
            move(0+r, getY());
            velocity.setX(velocity.getX() * -1);
        }
        if(pos.getX() >= flatPhysics.HEIGHT - r){
            move(flatPhysics.HEIGHT - r, getY());
            velocity.setX(velocity.getX() * -1);
        }
        if(pos.getY() <= 0 + r){
            move(getX(), 0+r);
            velocity.setY(velocity.getY() * -1);
        }
        if(pos.getY() >= flatPhysics.HEIGHT - r){
            move(getX(), flatPhysics.HEIGHT - r);
            velocity.setY(velocity.getY() * -1);
        }
//        if(pos.getX() <= 0 + r || pos.getX() >= 500 - r){
//            velocity.setX(velocity.getX() * -1);
//            System.out.println("WALLX");
//        }
//        if(pos.getY() <= 0 + r || pos.getY() >= 500 - r){
//            velocity.setY(velocity.getY() * -1);
//            System.out.println("WALL-E");
//        }
}

public boolean colliding(Ball b){
    return Equations.distance(pos, b.pos) <= r + b.r;
}
public void resolve(Ball b){
    CanvasHandler.drawTouching(this, b);
    Collisions.resolve(this, b);
}

    public void update(ArrayList<Ball> balls) {
        if(flatPhysics.Gravity)
            gravity();
        if(velocity.getMagnitude() <= 0.01)
            velocity = Vector.zeroVector;
        handleAcceleration();
        if(!balls.isEmpty()) {
            for (Ball b : balls) {
                if (this != b && colliding(b)) {
                    resolve(b);
                }
            }
        }
        handleVelocity();
    }

    private void gravity() {
        acceleration = acceleration.addition(G);
    }

    private void handleAcceleration() {
            velocity = velocity.addition(acceleration);
            setAcceleration(Vector.zeroVector);
        }

        private void handleVelocity(){
            pos = pos.addition(velocity.multiply(dt));
            velocity = velocity.multiply(restitution);
            WallCollision();
        }

        public void move(double x, double y){
        pos.setX((float)x);
        pos.setY((float)y);
        }

    public void push(double x, double y){
        pos.setX((float) (pos.getX() + x));
        pos.setY((float) (pos.getY() + y));
    }

    public void drawLine(GraphicsContext gc, ArrayList<Ball> balls, Text dist) {
        gc.setLineWidth(2);
        if (balls.isEmpty()) {
            dist.setText("0");
        } else {
            StringBuilder distances = new StringBuilder();
            for (Ball b : balls) {
                gc.strokeLine(pos.getX(),pos.getY(), b.pos.getX(), b.pos.getY());
            }
        }
    }

    public double getRestitution() {
        return restitution;
    }

    public Vector getVelocity() {
        return velocity;
    }

    public float getInverseMass() {
        return inverseMass;
    }

    public float getMass() {
        return mass;
    }
}

package com.example.flatphysics;

public class Equations {

    public static float distance(Vector pos1,Vector pos2){
        return (float) Math.sqrt(Math.pow(pos1.getX() - pos2.getX(),2) + Math.pow(pos1.getY() - pos2.getY(),2));
    }

    public static Vector getCenter(float x, float y, float r){
        return new Vector(x + r/2, y + r/2);
    }
}

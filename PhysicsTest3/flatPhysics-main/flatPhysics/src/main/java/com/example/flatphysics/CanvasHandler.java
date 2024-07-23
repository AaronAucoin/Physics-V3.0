package com.example.flatphysics;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;

import java.util.ArrayList;

public class CanvasHandler {
    private Canvas canvas;
    private ArrayList<Ball> balls;
    private static GraphicsContext gc;
    public static Ball draggedBall = null;
    private double mouseX, mouseY;
    private Vector newVel;

    public CanvasHandler(Canvas canvas, ArrayList<Ball> balls, GraphicsContext gc) {
        this.canvas = canvas;
        this.balls = balls;
        this.gc = gc;
        addEventHandlers();
    }

    public static void drawNormal(Vector[] points) {
        gc.setStroke(Color.BLACK);
        gc.strokeLine(points[0].getX(), points[0].getY(), points[1].getX(),points[1].getY());
    }

    private void addEventHandlers(){
        canvas.setOnMousePressed(this::handleMousePressed);
        canvas.setOnMouseDragged(this::handleMouseDragged);
        canvas.setOnMouseReleased(this::handleMouseReleased);
    }

    private void handleMousePressed(MouseEvent e) {
        mouseX = e.getX();
        mouseY = e.getY();

        draggedBall = findBall(mouseX, mouseY);
    }

    private void handleMouseDragged(MouseEvent e) {
        if (draggedBall != null) {
            mouseX = e.getX();
            mouseY = e.getY();
            if(e.getButton() == MouseButton.PRIMARY) {
                draggedBall.setVelocity(Vector.zeroVector);
                draggedBall.move(mouseX, mouseY);
            }
            else if(e.getButton() == MouseButton.SECONDARY){
                drawForceLine(draggedBall, mouseX, mouseY);
                newVel = new Vector((float) (draggedBall.getX() - mouseX), (float) (draggedBall.getY() - mouseY));
            }

//            float dx = (float) (e.getX() - draggedBall.getX());
//            float dy = (float) (e.getY() - draggedBall.getY());
//            flatPhysics.mouseLine(e.getX(), e.getY(), draggedBall.getPos(), gc);
//            Vector fa = new Vector(dx, dy);
//            draggedBall.setAcceleration(fa.multiply(Ball.dt));
//            // draggedBall.move(mouseX, mouseY);
//            mouseX = e.getX();
//            mouseY = e.getY();
        }
    }

    private void handleMouseReleased(MouseEvent e) {
        draggedBall.setVelocity(newVel);
        newVel = Vector.zeroVector;
        draggedBall = null;
    }

    private Ball findBall(double mouseX, double mouseY) {
        for (Ball b : balls) {
            if (b.contains(mouseX, mouseY))
                return b;
        }
        return null;
    }

    public static void drawTouching(Ball a, Ball b){
        gc.setStroke(Color.RED);
        gc.setLineWidth(3);
        gc.strokeLine(a.getX(), a.getY(), b.getX(),b.getY());
    }

    public static void drawForceLine(Ball draggedBall, double mouseX, double mouseY){
        gc.setStroke(Color.TOMATO);
        gc.setLineWidth(5);
        gc.strokeLine(draggedBall.getX(), draggedBall.getY(), mouseX,mouseY);
    }
}

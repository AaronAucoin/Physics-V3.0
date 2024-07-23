package com.example.flatphysics;

public class Collisions {

    private static float dx, dy;
    public static void resolve(Ball ballA, Ball ballB) {
        if (ballA == null || ballB == null) {
            throw new IllegalArgumentException("Balls cannot be null");
        }

        float distance = Equations.distance(ballA.getPos(), ballB.getPos());

        // Check if balls are overlapping
        float radiiSum = ballA.getr() + ballB.getr();
        if (distance < radiiSum) {
            resolveStatic(ballA, ballB, distance, radiiSum);
            resolveElastic(ballA, ballB);
        }
    }

    private static void resolveElastic(Ball a, Ball b){
        float distance = Equations.distance(a.getPos(),b.getPos());
        if (distance == 0) {
            // Avoid division by zero in normal calculation
            distance = 0.001f;
        }

        Vector normal = new Vector((b.getX() - a.getX()) / distance, (b.getY() - a.getY()) / distance);
        Vector tangent = new Vector(-normal.getY(), normal.getX());

        //dot product
        float dpTan1 = a.velocity.dot(tangent);
        float dpTan2 = b.velocity.dot(tangent);

        //dot product normal
        float dpNorm1 = a.velocity.dot(normal);
        float dpNorm2 = b.velocity.dot(normal);

        //conservation of momentum 1D
        float m1 = (dpNorm1 * (a.getMass() - b.getMass()) + 2f * b.getMass() * dpNorm2) / (a.getMass() + b.getMass());
        float m2 = (dpNorm2 * (b.getMass() - a.getMass()) + 2f * a.getMass() * dpNorm1) / (a.getMass() + b.getMass());

        System.out.println("Mass of a + b: " + a.getMass() + b.getMass());

        a.velocity.setX(tangent.getX() * dpTan1 + normal.getX() * m1);
        a.velocity.setY(tangent.getY() * dpTan1 + normal.getY() * m1);
        b.velocity.setX(tangent.getX() * dpTan2 + normal.getX() * m2);
        b.velocity.setY(tangent.getY() * dpTan2 + normal.getY() * m2);

        // Debug logging
        System.out.println("Elastic resolution: ");
        System.out.println("Ball A velocity: x=" + a.velocity.getX() + ", y=" + a.velocity.getY());
        System.out.println("Ball B velocity: x=" + b.velocity.getX() + ", y=" + b.velocity.getY());
        System.out.println("Ball A Position: " + a.getPos().toString());

//        Vector bNewV = new Vector(-b.velocity.getY(), b.velocity.getX());
//        Vector aNewV = new Vector(-a.velocity.getX(), a.velocity.getY());
    }

    private static void resolveStatic(Ball ballA, Ball ballB, float distance, float radiiSum) {
        if (distance == 0) {
            // Handle the case where the balls are exactly at the same position
            // For simplicity, push them apart along the x-axis
            float overlap = 0.5f * radiiSum;
            ballA.push(overlap, 0);
            ballB.push(-overlap, 0);
            return;
        }

        float overlap = 0.5f * (radiiSum - distance);

        dx = (ballA.getX() - ballB.getX()) / distance;
        dy = (ballA.getY() - ballB.getY()) / distance;

        ballA.push(overlap * dx, overlap * dy);
        ballB.push(-overlap * dx, -overlap * dy);
    }
}

package bouncing_balls;

/**
 * The physics model.
 * <p>
 * This class is where you should implement your bouncing balls model.
 * <p>
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 *
 * @author Simon Robillard
 */
class Model {

    public final double GRAVITATION = 9.82;
    double areaWidth, areaHeight;

    Ball[] balls;

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[2];
        balls[0] = new Ball(width / 1, height * 0.7, -0.1, 0, 0.9, 4);
        balls[1] = new Ball(width / 3, height * 0.7, 0.8, 0, 0.2, 4);
    }

    void step(double deltaT) {
        boolean canCollide = true;
        if (Math.sqrt(Math.pow((balls[0].x - balls[1].x), 2) + Math.pow((balls[0].y) - (balls[1].y), 2)) <= balls[0].radius + balls[1].radius) {
            ballCollision(balls[0], balls[1]);
            canCollide = false;
        }
        for (Ball b : balls) {
            double vyBefore = b.vy;

            if(canCollide) {
                if ((b.x < b.radius))
                    b.vx = Math.abs(b.vx);
                else if (b.x > areaWidth - b.radius)
                    b.vx = -Math.abs(b.vx);

                if ((b.y < b.radius))
                    b.vy = Math.abs(b.vy);
                else if (b.y > areaHeight - b.radius)
                    b.vy = -Math.abs(b.vy);
            }

            // add gravity to our balls - important to not have it added when bouncing at the bottom since there would otherwise
            // be extra speed added at each bounce.
            if (!(vyBefore != b.vy))
                b.vy -= deltaT * GRAVITATION;

            // compute new position according to the speed of the ball
            b.x += deltaT * b.vx;
            b.y += deltaT * b.vy;

            // prints the total energy in the system so that we can test the legitimacy of out calculations
            System.out.println("Total energy: " + (kineticEnergyOfSystemTest(balls) + potentialEnergyOfSystemTest(balls)));

        }
    }

    private double potentialEnergyOfSystemTest(Ball... balls) {
        double totalPotentialEnergy = 0;
        for (Ball ball : balls) {
            totalPotentialEnergy += ball.mass * ball.y * GRAVITATION;
        }

        return totalPotentialEnergy;
    }

    private double kineticEnergyOfSystemTest(Ball... balls) {
        double totalKineticEnergy = 0;
        for (Ball ball : balls) {
            totalKineticEnergy += ball.mass * Math.pow(ball.vx, 2) / 2 + ball.mass * Math.pow(ball.vy, 2) / 2;
        }

        return totalKineticEnergy;
    }

    double[] collision_vector(Ball b1, Ball b2) {
        double collision_vs[] = new double[2];
        double vx = (b1.x - b2.x);
        double vy = (b1.y) - (b2.y);
        collision_vs[0] = vx;
        collision_vs[1] = vy;
        return collision_vs;
    }

    private void ballCollision(Ball ball1, Ball ball2) {
        //Collision vector that we turn in to the new base-vector for x', y'.
        double[] xPrimBasisVector = collision_vector(ball1, ball2);
        double[] yPrimBasisVector = new double[]{0, 0};
        yPrimBasisVector[0] = -xPrimBasisVector[1];
        yPrimBasisVector[1] = xPrimBasisVector[0];


        //Transform old speed-vectors to new coordinate system with invers. We calculate A^-1 below:
        double det = 1 / (xPrimBasisVector[0] * yPrimBasisVector[1] - (yPrimBasisVector[0] * xPrimBasisVector[1]));
        double[] xPrimInvers = new double[]{det * (yPrimBasisVector[1]), det * (-xPrimBasisVector[1])};
        double[] yPrimInvers = new double[]{det * (-yPrimBasisVector[0]), det * (xPrimBasisVector[0])};

        //Transform old speed-vectors to new coordinate system with matrix-multiplication:
        double[] ball1VelocityTransformed = new double[]{xPrimInvers[0] * ball1.vx + yPrimInvers[0] * ball1.vy, xPrimInvers[1] * ball1.vx + yPrimInvers[1] * ball1.vy};
        double[] ball2VelocityTransformed = new double[]{xPrimInvers[0] * ball2.vx + yPrimInvers[0] * ball2.vy, xPrimInvers[1] * ball2.vx + yPrimInvers[1] * ball2.vy};


        //Calculate the new speeds in the new coordinates - 2 linear equations:
        double ball1XVelocity = (ball1.mass * ball1VelocityTransformed[0] + ball2.mass * ball2VelocityTransformed[0] - ball2.mass * (ball1VelocityTransformed[0] - ball2VelocityTransformed[0])) / (ball2.mass + ball1.mass);
        double ball2XVelocity = (ball1VelocityTransformed[0] - ball2VelocityTransformed[0] + ball1XVelocity);

        ball1VelocityTransformed[0] = ball1XVelocity;
        ball2VelocityTransformed[0] = ball2XVelocity;


        //Rotate back the new speed vectors:
        double[] ball1VelocityReturned = new double[]{xPrimBasisVector[0] * ball1VelocityTransformed[0] + yPrimBasisVector[0] * ball1VelocityTransformed[1], xPrimBasisVector[1] * ball1VelocityTransformed[0] + yPrimBasisVector[1] * ball1VelocityTransformed[1]};
        double[] ball2VelocityReturned = new double[]{xPrimBasisVector[0] * ball2VelocityTransformed[0] + yPrimBasisVector[0] * ball2VelocityTransformed[1], xPrimBasisVector[1] * ball2VelocityTransformed[0] + yPrimBasisVector[1] * ball2VelocityTransformed[1]};


        //Giv our balls the new speeds:
        ball1.vx = ball1VelocityReturned[0];
        ball1.vy = ball1VelocityReturned[1];
        ball2.vx = ball2VelocityReturned[0];
        ball2.vy = ball2VelocityReturned[1];
    }

    /**
     * Simple inner class describing balls.
     */
    class Ball {

        Ball(double x, double y, double vx, double vy, double r, double mass) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.radius = r;
            this.mass = mass;
        }

        /**
         * Position, speed, and radius of the ball. You may wish to add other attributes.
         */
        double x, y, vx, vy, radius, mass;
    }
}

package bouncing_balls;

/**
 * The physics model.
 * 
 * This class is where you should implement your bouncing balls model.
 * 
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 * 
 * @author Simon Robillard
 *
 */
class Model {


	double[] collision_vector(Ball b1, Ball b2){
		double collision_vs[] = new double[2];
		double vx = (b1.x - b2.x);
		double vy = (b1.y) - (b2.y);
		collision_vs[0] = vx;
		collision_vs[1] = vy;
		return collision_vs;
	}
	
	double[] give_components(Ball b, double[] coords){
		/* x (coord 0, coord 1)
		 * y (coord 1, -coord 0)
		 * x' [-coord0 -coord1] [x1]
		 * y' [-coord1 coord0] [y1]
		 */

		double[] vs = new double[2];
		double vx = (b.vx * coords[0] + b.vy*coords[1]);
		double vy = (b.vx *(coords[1]) + b.vy*-coords[0]);
		vs[0] = vx;
		vs[1] = vy;
		return vs;
	}

	void collison(Ball b1, Ball b2){
		/* u2 - u1 = -(v2 - v1)
		 * 
		 */
		double[] coords = collision_vector(b1, b2);
		double[] b1_new_v_vector = give_components(b1, coords);
		double[] b2_new_v_vector = give_components(b2, coords);
		
		b1_new_v_vector[0] = -b1_new_v_vector[0];

		b2_new_v_vector[0] = -b2_new_v_vector[0];

		double det = 1/(coords[0]*-coords[0] - (coords[1]*coords[1]));

		b1_new_v_vector[0] = det*(b1_new_v_vector[0]*-coords[0] + b1_new_v_vector[1]*-coords[1]);

		b2_new_v_vector[0] = det*(b2_new_v_vector[0]*-coords[1] + b2_new_v_vector[1]*coords[0]);

		b1_new_v_vector[1] = det*(b1_new_v_vector[0]*-coords[0] + b1_new_v_vector[1]*-coords[1]);

		b2_new_v_vector[1] = det*(b2_new_v_vector[0]*-coords[1] + b2_new_v_vector[1]*coords[0]);
		
		b1.vx = b1_new_v_vector[0];
		b1.vy = b1_new_v_vector[1];

		b2.vx = b2_new_v_vector[0];
		b2.vy = b2_new_v_vector[1];

		/* [-coord0 -coord1]
		 * [-coord1 coord0]
		 */

		/*double det = 1/(coords[0]*-coords[0] - (coords[1]*coords[1]));
		double vx1 = b1.vx;
		double vx2 = b2.vx;
		b1.vx = det*(b1.vx*-coords[0] + b1.vy*-coords[1]);
		b1.vy = det*(vx1*-coords[1] + b1.vy*coords[0]);
		b2.vx = det*(b2.vx*-coords[0] + b2.vy*-coords[1]);
		b2.vy = det*(vx2*-coords[1] + b2.vy*coords[0]);*/
	}

	double areaWidth, areaHeight;
	
	Ball [] balls;

	Model(double width, double height) {
		areaWidth = width;
		areaHeight = height;
		
		// Initialize the model with a few balls
		balls = new Ball[2];
		balls[0] = new Ball(width / 3, height * 0.9, 0.9, 3.6, 0.2, 1000);
		balls[1] = new Ball(2 * width / 3, height * 0.7, -1.3, 1.6, 0.3, 10);
	}

	void step(double deltaT) {
		// TODO this method implements one step of simulation with a step deltaT
		if(Math.sqrt(Math.pow((balls[0].x - balls[1].x), 2) + Math.pow((balls[0].y) - (balls[1].y), 2)) <= balls[0].radius + balls[1].radius + 0.002){
			collison2(balls[0], balls[1]);
		}
		for (Ball b : balls) {
			// detect collision with the border
			if (b.x < b.radius || b.x > areaWidth - b.radius) {
				b.vx *= -1; // change direction of ball
			}
			if (b.y < b.radius || b.y > areaHeight - b.radius) {
				b.vy *= -1;
			}

			// add gravity to our balls
			b.vy -= deltaT * 9.82;

			// compute new position according to the speed of the ball
			b.x += deltaT * b.vx;
			b.y += deltaT * b.vy;
		}
	}

	private void collison2(Ball ball1, Ball ball2) {
		//Collision vector that we turn in to the new base-vector for x', y'.
		double[] xPrimBasisVector = collision_vector(ball1, ball2);
		double[] yPrimBasisVector = new double[]{0,0};
		yPrimBasisVector[0] = -xPrimBasisVector[1];
		yPrimBasisVector[1] = xPrimBasisVector[0];


		//Transform old speed-vectors to new coordinate system with invers. We calculate A^-1 below:
		double det = 1/(xPrimBasisVector[0]*yPrimBasisVector[1] - (yPrimBasisVector[0]*xPrimBasisVector[1]));
		double[] xPrimInvers = new double[]{det*(yPrimBasisVector[1]), det*(-xPrimBasisVector[1])};
		double[] yPrimInvers = new double[]{det*(-yPrimBasisVector[0]), det*(xPrimBasisVector[0])};

		//Transform old speed-vectors to new coordinate system:
		double[] ball1VelocityTransformed = new double[]{xPrimInvers[0]*ball1.vx + yPrimInvers[0]*ball1.vy, xPrimInvers[1]*ball1.vx + yPrimInvers[1]*ball1.vy};
		double[] ball2VelocityTransformed = new double[]{xPrimInvers[0]*ball2.vx + yPrimInvers[0]*ball2.vy, xPrimInvers[1]*ball2.vx + yPrimInvers[1]*ball2.vy};


		//Calculate the new speeds in the new coordinates - 2 linear equations:
		double ball1XVelocity = (ball1.mass*ball1VelocityTransformed[0] + ball2.mass*ball2VelocityTransformed[0] - ball2.mass*(ball1VelocityTransformed[0]- ball2VelocityTransformed[0]))/(ball2.mass + ball1.mass);
		double ball2XVelocity = (ball1VelocityTransformed[0] - ball2VelocityTransformed[0] + ball1XVelocity);

		ball1VelocityTransformed[0] = ball1XVelocity;
		ball2VelocityTransformed[0] = ball2XVelocity;
		//ball1VelocityTransformed[1] *= -1;
		//ball2VelocityTransformed[1] *= -1;


		//Rotate back the new speed vectors:
		double[] ball1VelocityReturned = new double[]{xPrimBasisVector[0]*ball1VelocityTransformed[0] + yPrimBasisVector[0]*ball1VelocityTransformed[1], xPrimBasisVector[1]*ball1VelocityTransformed[0] + yPrimBasisVector[1]*ball1VelocityTransformed[1]};
		double[] ball2VelocityReturned = new double[]{xPrimBasisVector[0]*ball2VelocityTransformed[0] + yPrimBasisVector[0]*ball2VelocityTransformed[1], xPrimBasisVector[1]*ball2VelocityTransformed[0] + yPrimBasisVector[1]*ball2VelocityTransformed[1]};


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

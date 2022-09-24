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

	double get_angle(Ball b1, Ball b2){
		double dx = b1.x - b2.x;
		double dy = b1.y - b2.y;
		double angle = Math.PI/2;//initilized to pi/2 because if dy is zero, no case will match and we want to return pi/2
		if(dy == 0){
			angle = 0;
		}else if((dx > 0 && dy >= 0) || (dx < 0 && dy < 0)){
			angle = Math.atan(dy/dx); // In third and the the first quadrant
		}else if(dx < 0 && dy > 0){
			angle = Math.PI + Math.atan(dy/dx);// In the second quadrant
		}else if(dx > 0 && dy < 0){
			angle = 2*Math.PI + Math.atan(dy/dx); //In the fourth quadrant
		}
		return angle;
	}

	double[][] find_transform_matrix(double theta){
		double[][] transform_matrix = new double[2][2];
		double cos_theta = Math.cos(theta);
		double sin_theta = Math.sin(theta);
		transform_matrix[0][0] = cos_theta; transform_matrix[0][1] = -sin_theta;
		transform_matrix[1][0] = sin_theta; transform_matrix[1][1] = cos_theta;
		return transform_matrix;
	}

	double[][] find_inverse_matrix(double[][] transform_matrix){
		double[][] inverse_matrix = new double[2][2];
		double det = 1/((transform_matrix[0][0]*transform_matrix[1][1]) - (transform_matrix[0][1]*transform_matrix[1][0]));
		inverse_matrix[0][0] = det*transform_matrix[1][1]; inverse_matrix[0][1] = -det*transform_matrix[0][1];
		inverse_matrix[1][0] = -det*transform_matrix[1][0]; inverse_matrix[1][1] = det*transform_matrix[0][0];
		return inverse_matrix;
	}

	double[] transform_to_new_coordinates(double[] v, double[][] inverse_matrix){
		double[] new_v = new double[2];
		new_v[0] = v[0]*inverse_matrix[0][0] + v[1]*inverse_matrix[0][1];
		new_v[1] = v[0]*inverse_matrix[1][0] + v[1]*inverse_matrix[1][1];
		return new_v;
	}

	double[] return_to_old_coordinates(double[] v, double[][] transform_matrix){
		double[] new_v = new double[2];
		new_v[0] = v[0]*transform_matrix[0][0] + v[1]*transform_matrix[0][1];
		new_v[1] = v[0]*transform_matrix[1][0] + v[1]*transform_matrix[1][1];
		return new_v;
	}

	double[] calculate_velocity(double mass_b1, double mass_b2, double v1, double v2){
		double total_momentum = mass_b1*v1 + mass_b2*v2;
		double relative_velocity = v2 - v1;
		double velocity_after_collision_b1 = (total_momentum + mass_b2*relative_velocity)/(mass_b1 + mass_b2);
		double velocity_after_collision_b2 = velocity_after_collision_b1 - relative_velocity;
		double[] v1_v2 = {velocity_after_collision_b1, velocity_after_collision_b2};
		return v1_v2;
	}

	void handle_collision(Ball b1, Ball b2){
		double theta = get_angle(b1, b2);
		double[][] transform_matrix = find_transform_matrix(theta);
		double[][] inverse_matrix = find_inverse_matrix(transform_matrix);
		double[] b1_v = {b1.vx, b1.vy};
		double[] b2_v = {b2.vx, b2.vy};
		double[] b1_v_prim = transform_to_new_coordinates(b1_v, inverse_matrix);
		double[] b2_v_prim = transform_to_new_coordinates(b2_v, inverse_matrix);

		double[] new_velocities = calculate_velocity(b1.mass, b2.mass, b1_v_prim[0], b2_v_prim[0]);

		b1_v_prim[0] = new_velocities[0];
		b2_v_prim[0] = new_velocities[1];

		b1_v = return_to_old_coordinates(b1_v_prim, transform_matrix);
		b2_v = return_to_old_coordinates(b2_v_prim, transform_matrix);

		b1.vx = b1_v[0]; b1.vy = b1_v[1];
		b2.vx = b2_v[0]; b2.vy = b2_v[1];
	}

	
	double areaWidth, areaHeight;
	
	Ball [] balls;

	Model(double width, double height) {
		areaWidth = width;
		areaHeight = height;
		
		// Initialize the model with a few balls
		balls = new Ball[2];
		balls[0] = new Ball(width - 0.2, height - 0.5, 1, 1.6, 0.2,100);
		balls[1] = new Ball(width - 0.6, height - 0.3, -3,2, 0.2,3);
	}
	boolean check = true;
	void step(double deltaT) {
		if(check && Math.sqrt(Math.pow((balls[0].x - balls[1].x), 2) + Math.pow((balls[0].y) - (balls[1].y), 2)) <= balls[0].radius + balls[1].radius){
			handle_collision(balls[0], balls[1]);
			check = false;
			return;
		}
		for (Ball b : balls) {
			// detect collision with the border
			if (b.x <= b.radius){
				b.vx = Math.abs(b.vx);
				check = true;
			}else if(b.x >= areaWidth - b.radius){
				b.vx = -Math.abs(b.vx);
				check = true;
			}
			
			if (b.y <= b.radius){
				b.vy = Math.abs(b.vy);
				b.vy -= deltaT * 9.82;
				check = true;
			}else if(b.y >= areaHeight - b.radius){
				b.vy = -Math.abs(b.vy);
				b.vy -= deltaT * 9.82;
				check = true;
			}
			
			// compute new position according to the speed of the ball
			b.x += deltaT * b.vx;
			b.y += deltaT * b.vy;
			b.vy -= deltaT * 9.82;
		}
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

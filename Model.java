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
	//returns the angle theta which the balls are colliding with. We use b2 as the reference, so b1 rotates around b2, and b2 acts like the
	//origo. We take to count all the possible cases that b1 can exist in. 
	double get_angle(Ball b1, Ball b2){
		double dx = b1.x - b2.x;
		double dy = b1.y - b2.y;
		double angle = 0;
		if(dy == 0 && dx > 0){
			angle = 0;//no rotation because b1 is on the right side of the x axis already
		}else if(dy == 0 && dx < 0){
			angle = Math.PI;//half circle rotation because b1 is on the negative left side of the x axis
		}else if((dx > 0 && dy > 0)){
			angle = Math.atan(dy/dx);//b1 is in the first quadrant
		}else if((dx < 0 && dy < 0)){
			angle = Math.PI + Math.atan(dy/dx);//b1 is in the third quadrant
		}else if(dx < 0 && dy > 0){
			angle = Math.PI + Math.atan(dy/dx);//b1 is in the second quadrant
		}else if(dx > 0 && dy < 0){
			angle = 2*Math.PI + Math.atan(dy/dx);//b1 is in the fourth quadrant
		}else if(dx == 0 && dy > 0){
			angle = Math.PI/2;//b1 is on the positive y axis
		}else if(dx == 0 && dy < 0){
			angle = 1.5*Math.PI;//b1 is on the negative y axis
		}
		return angle;
	}
	//computes the matrix for the transformation from the rotated coordinate system to the old one. 
	double[][] find_transform_matrix(double theta){
		double[][] transform_matrix = new double[2][2];
		double cos_theta = Math.cos(theta);
		double sin_theta = Math.sin(theta);
		transform_matrix[0][0] = cos_theta; transform_matrix[0][1] = -sin_theta;
		transform_matrix[1][0] = sin_theta; transform_matrix[1][1] = cos_theta;
		return transform_matrix;
	}
	//Computes the matrix for the transformation from the old coordinate system to the rotated one
	double[][] find_inverse_matrix(double[][] transform_matrix){
		double[][] inverse_matrix = new double[2][2];
		double det = 1/((transform_matrix[0][0]*transform_matrix[1][1]) - (transform_matrix[0][1]*transform_matrix[1][0]));
		inverse_matrix[0][0] = det*transform_matrix[1][1]; inverse_matrix[0][1] = -det*transform_matrix[0][1];
		inverse_matrix[1][0] = -det*transform_matrix[1][0]; inverse_matrix[1][1] = det*transform_matrix[0][0];
		return inverse_matrix;
	}
	//computes the vectors in the rotated coordinate system
	double[] transform_to_new_coordinates(double[] v, double[][] inverse_matrix){
		double[] new_v = new double[2];
		new_v[0] = v[0]*inverse_matrix[0][0] + v[1]*inverse_matrix[0][1];
		new_v[1] = v[0]*inverse_matrix[1][0] + v[1]*inverse_matrix[1][1];
		return new_v;
	}
	//translates the new computed vectors to the old coordinate system
	double[] return_to_old_coordinates(double[] v, double[][] transform_matrix){
		double[] new_v = new double[2];
		new_v[0] = v[0]*transform_matrix[0][0] + v[1]*transform_matrix[0][1];
		new_v[1] = v[0]*transform_matrix[1][0] + v[1]*transform_matrix[1][1];
		return new_v;
	}
	//the function that calculate the velocity using the formulas
	double[] calculate_velocity(double mass_b1, double mass_b2, double v1, double v2){
		double total_momentum = mass_b1*v1 + mass_b2*v2;
		double relative_velocity = v2 - v1;
		double velocity_after_collision_b1 = (total_momentum + mass_b2*relative_velocity)/(mass_b1 + mass_b2);
		double velocity_after_collision_b2 = velocity_after_collision_b1 - relative_velocity;
		double[] v1_v2 = {velocity_after_collision_b1, velocity_after_collision_b2};
		return v1_v2;
	}
	//to check if the balls are moving apart when they are touching each others so we don't calculate any collisions
	boolean check_if_moving_apart(double v1, double v2){
		boolean check = (v1 >= 0) && (v2 <= 0);
		//because we are flipping the coordinates with angle theta so they are aligned with
		//the collison vector, the ball b1 is always in the first quadrant, and therefore, if it
		//has a positive speed, and b2 has negative speed, then they are moving apart, and are not colliding
		return check;
	}
	//check if the balls are moving in the same directions, but the one in front is faster than the one behind, this will happen if the 
	//balls start touching each others from the beginning.
	boolean check_if_moving_in_the_same_direction_but_not_colliding(double v1, double v2){
		/*
		 * because we have the coordinates flipped to fit with b1, we can say that if both v1 and v2 are less than zero,
		 * then they are moving on the same direction which is negative, and if v1 is greater than v2, then v2 is moving
		 * faster on the negative direction, therefore they are not colliding. If they both are positive, then they are
		 * moving on the positive direction, and if v1 is greater, then b1 is moving faster, and they are not colliding
		 */
		boolean check = (v1 > 0 && v2 > 0 && v2 < v1) || (v1 < 0 && v2 < 0 && v2 < v1);	
		return check;
	}

	void handle_collision(Ball b1, Ball b2){
		double theta = get_angle(b1, b2);
		double[][] transform_matrix = find_transform_matrix(theta);
		double[][] inverse_matrix = find_inverse_matrix(transform_matrix);
		double[] b1_v = {b1.vx, b1.vy};
		double[] b2_v = {b2.vx, b2.vy};
		double[] b1_v_prim = transform_to_new_coordinates(b1_v, inverse_matrix);
		double[] b2_v_prim = transform_to_new_coordinates(b2_v, inverse_matrix);

		if(check_if_moving_apart(b1_v_prim[0], b2_v_prim[0])
		||check_if_moving_in_the_same_direction_but_not_colliding(b1_v_prim[0], b2_v_prim[0]))
			return;

		double[] new_velocities = calculate_velocity(b1.mass, b2.mass, b1_v_prim[0], b2_v_prim[0]);

		b1_v_prim[0] = new_velocities[0];
		b2_v_prim[0] = new_velocities[1];

		b1_v = return_to_old_coordinates(b1_v_prim, transform_matrix);
		b2_v = return_to_old_coordinates(b2_v_prim, transform_matrix);
		System.out.println("momentum on x before: " + (b1.vx*b1.mass + b2.vx*b2.mass));
		System.out.println("momentum on y before: " + (b1.vy*b1.mass + b2.vy*b2.mass));
		b1.vx = b1_v[0]; b1.vy = b1_v[1];
		b2.vx = b2_v[0]; b2.vy = b2_v[1];
		System.out.println("momentum on x after: " + (b1.vx*b1.mass + b2.vx*b2.mass));
		System.out.println("momentum on y after: " + (b1.vy*b1.mass + b2.vy*b2.mass));
	}

	
	double areaWidth, areaHeight;
	
	Ball [] balls;

	Model(double width, double height) {
		areaWidth = width;
		areaHeight = height;
		
		// Initialize the model with a few balls
		balls = new Ball[2];
		//some testcases, those between the stars used to fail miserably in the beginning when we first started
        //*
		//special case when both are on the ground, with opposite directions
		//balls[0] = new Ball(width - 0.2, 0.2, 3,0, 0.2,3);
        //balls[1] = new Ball(width - 0.6, 0.2, -1,0, 0.2,3);
        //balls[0] = new Ball(width - 0.2, 0.2, 1, 0, 0.2,3);
        //balls[1] = new Ball(width - 0.6, 0.2, -3,0, 0.2,3);
		//a test case where they used to get stuck after the sixth or fifth collison
		//balls[0] = new Ball(width/3, height*0.7, 1, 4, 0.2,3);
        //balls[1] = new Ball(width/2, height*0.2, 1.6,0, 0.2,4);
		//*
		
		//test case with very different masses
		//balls[0] = new Ball(width/2, height*0.7, 0, 4, 0.2,100);
        //balls[1] = new Ball(width/2, height*0.2, 0,0, 0.2,4);
		
		
		//test case when the x coordinates are the same
		//balls[0] = new Ball(width/3, height*0.7, 1, 4, 0.2,100);
        //balls[1] = new Ball(width/2, height*0.2, 1.6,0, 0.2,4);

		//test case for the diagonal when balls are colliding
		//balls[0] = new Ball(width - 1, height/2, -4, 4, 0.2,4);
        //balls[1] = new Ball(width - 1.2, height/2 + 0.3, 4,-4, 0.2,4);
		
		
		//test case for the diagonal when balls are moving apart, the balls will get stuck togethen in this case if 
		//the chcek in the function handle collision is taken away.
		//balls[0] = new Ball(width - 1, height/2, 4, -4, 0.2,4);
        //balls[1] = new Ball(width - 1.2, height/2 + 0.3, -4,4, 0.2,4);

		//an arbitrary input
		balls[0] = new Ball(0.9, height/2,5, 3, 0.2,4);
        balls[1] = new Ball(2, height/3, 6,-4, 0.4,4);
	}

	void step(double deltaT) {
		if(Math.sqrt(Math.pow((balls[0].x - balls[1].x), 2) + Math.pow((balls[0].y) - (balls[1].y), 2)) <= balls[0].radius + balls[1].radius){
			handle_collision(balls[0], balls[1]);
		}
		for (Ball b : balls) {
			// detect collision with the border
			if (b.x <= b.radius){
				b.vx = Math.abs(b.vx);
			}else if(b.x >= areaWidth - b.radius){
				b.vx = -Math.abs(b.vx);
			}
			
			if (b.y <= b.radius){
				b.vy = Math.abs(b.vy);
				b.vy -= deltaT * 9.82;
			}else if(b.y >= areaHeight - b.radius){
				b.vy = -Math.abs(b.vy);
				b.vy -= deltaT * 9.82;
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

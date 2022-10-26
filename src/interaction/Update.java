package interaction;

/**
 * @author Mattia Crispino
 */

import java.awt.Point;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.ArrayList;

import it.unipr.sowide.actodes.registry.Reference;

/**
*
* The {@code Update} class defines an action used for asking a Boid
* to update it's position in relation with its neighborhood.
**/

public class Update implements Serializable {
	
	private static final long serialVersionUID = 1L;
	private double direction;
	private int round;
	private Point position;
	private Point2D.Double velocity;
	private Reference senderReference;
	private ArrayList<Point> weightEdgePRM;

	/**
	 * Class constructor
	 * @param ref		reference of the boid
	 * @param pos		position of the boid
	 * @param v			velocity of the boid
	 * @param dir		direction of the boid
	 * @param round		actual round of the boid's position update
	 * @param edge		the edge whose weight needs to be incremented by 1 (for COVERING behaviour -if selected-)
	 */
	public Update(Reference ref, Point pos, Point2D.Double v, double dir, int round, ArrayList<Point> edge) {
		this.senderReference = ref;
		this.position = pos;
		this.velocity = v;
		this.direction = dir;
		this.round = round;
		this.weightEdgePRM = edge;
	}

	public Reference getSenderReference() {
		return this.senderReference;
	}
	
	@Override
	public String toString() {
		return "Update [position=" + position + ", velocity=" + velocity + ", direction=" + direction
				+ ", round=" + round
				+ ", senderReference=" + senderReference + "]";
	}


	/**
	 * Get the position included in the message
	 * @return the position
	 */
	public Point getPosition() {
		return this.position;
	}

	/**
	 * Get the velocity include in the message
	 * @return the velocity
	 */
	public Point2D.Double getVelocity() {
		return this.velocity;
	}

	/**
	 * Get the direction included in the message
	 * @return the direction
	 */
	public double getDirection() {
		return this.direction;
	}

	/**
	 * Get the round of execution included in the message
	 * @return the round
	 */
	public int getRound() {
		return this.round;
	}
	
	/**
	 * Get the edge which weight needs to be incremented
	 * @return the edge as a list [starting_point, destination_point]
	 */
	public ArrayList<Point> getEdgeToIncrementPRM() {
		return this.weightEdgePRM;
	}
	
	
}

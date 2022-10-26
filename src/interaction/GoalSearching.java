package interaction;
/**
 * @author Mattia Crispino
 */

import java.awt.Point;
import java.awt.geom.Point2D;
import java.io.Serializable;

import it.unipr.sowide.actodes.registry.Reference;

/**
*
* The {@code GoalSearching} class defines a message that gives sent by the boid
* if the goal searching behaviour is choosen.
**/
public class GoalSearching implements Serializable {
	
	private static final long serialVersionUID = 1L;
	private Point position;
	private Point2D.Double velocity;
	private Reference senderReference;

	/**
	 * Class constructor
	 * @param ref		reference of the boid
	 * @param pos		position of the boid
	 * @param v			velocity of the boid
	 */
	public GoalSearching(Reference ref, Point pos, Point2D.Double v) {
		this.senderReference = ref;
		this.position = pos;
		this.velocity = v;
	}

	/**
	 * Get the reference of the sender
	 * @return the sender's reference
	 */
	public Reference getSenderReference() {
		return this.senderReference;
	}
	
	@Override
	public String toString() {
		return "GoalSearching [position=" + position + ", velocity=" + velocity
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


}

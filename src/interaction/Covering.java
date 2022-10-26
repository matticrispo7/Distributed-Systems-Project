package interaction;

/**
 * @author Mattia Crispino
 */

import java.awt.Point;
import java.io.Serializable;
import java.util.ArrayList;

import it.unipr.sowide.actodes.registry.Reference;

/**
*
* The {@code Covering} class defines an action used for asking a Boid
* to increment the weight of the edge passed in the message.
**/

public class Covering implements Serializable {
	
	private static final long serialVersionUID = 1L;
	private ArrayList<Point> edge;
	private Reference sender;
	
	/**
	 * Class constructor
	 * @param e		the edge which weight need to be updated
	 * @param ref	the sender's reference
	 */
	public Covering(ArrayList<Point> e, Reference ref) {
		this.edge = e;
		this.sender = ref;
	}

	/**
	 * Get the edge which weight needs to be incremented
	 * @return
	 */
	public ArrayList<Point> getEdge() {
		return this.edge;
	}

	/**
	 * Set the edge to send to the boids
	 * @param edge the list of the edge as [starting_point, destination_point]
	 */
	public void setEdge(ArrayList<Point> edge) {
		this.edge = edge;
	}
	
	/**
	 * Get the sender's reference
	 * @return the sender's reference
	 */
	public Reference getSenderReference() {
		return this.sender;
	}
	
	@Override
	public String toString() {
		return "Covering [edge=" + edge + "]";
	}

}

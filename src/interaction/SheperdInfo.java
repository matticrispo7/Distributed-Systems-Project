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
* The {@code SheperdInfo} class defines a message that gives to the boid
* the information (position and reference) about the sheperd (if the sheperding behaviour
* is chosen)
**/

public class SheperdInfo implements Serializable {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private Reference sheperdRef;
	private Point sheperdPos;
	private Point2D.Double sheperVel;
	
	/**
	 * Class constructor
	 * @param ref	the sheperd's reference
	 * @param pos	the sheperd's position
	 * @param vel	the sheeper's velocity
	 */
	public SheperdInfo(Reference ref, Point pos, Point2D.Double vel) {
		this.sheperdRef = ref;
		this.sheperdPos = pos;
		this.sheperVel = vel;
	}

	/**
	 * Get the sheperd reference
	 * @return the sheperd's reference
	 */
	public Reference getSheperdRef() {
		return this.sheperdRef;
	}

	/**
	 * Update the sheperd's  reference
	 * @param sheperdRef the reference of the sheperd
	 */
	public void setSheperdRef(Reference sheperdRef) {
		this.sheperdRef = sheperdRef;
	}

	/**
	 * Get the position of the sheperd
	 * @return the sheperd's position
	 */
	public Point getSheperdPos() {
		return this.sheperdPos;
	}

	/**
	 * Set the sheperd's position
	 * @param sheperdPos the position of the sheperd
	 */
	public void setSheperdPos(Point sheperdPos) {
		this.sheperdPos = sheperdPos;
	}

	/**
	 * Get the velocity of the sheperd
	 * @return the sheperd's velocity
	 */
	public Point2D.Double getSheperVel() {
		return this.sheperVel;
	}

	/**
	 * Set the velocity of the sheperd
	 * @param sheperVel the sheperd's velocity
	 */
	public void setSheperVel(Point2D.Double sheperVel) {
		this.sheperVel = sheperVel;
	}
	
	

}

package interaction;

/**
 * @author Mattia Crispino
 */

import it.unipr.sowide.actodes.interaction.Response;

/**
*
* The {@code Continue} enumeration is used for informing a boid 
* to continue it's execution by calling the method "moveBoid" (of the 
* class Boid.java) in order to move the boid to its next position.
*
* This enumeration has a single element: <code>CONTINUE</code> and it is used
* for proving a thread safe implementation of the singleton pattern.
*
**/

public enum Continue implements Response {
	  /**
	   * The singleton instance.
	   *
	  **/
	  CONTINUE;
}

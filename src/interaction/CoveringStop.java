package interaction;

/**
 * @author Mattia Crispino
 */

import it.unipr.sowide.actodes.interaction.Response;

/**
*
* The {@code CoveringStop} enumeration is used for informing all the boids that
* one of them have visited all the nodes in the map and so the execution need 
* to be stopped.
*
* This enumeration has a single element: <code>COVERING_STOP</code> and it is used
* for proving a thread safe implementation of the singleton pattern.
*
**/

public enum CoveringStop implements Response {
	  /**
	   * The singleton instance.
	   *
	  **/
	  COVERING_STOP;
}

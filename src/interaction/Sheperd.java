package interaction;
/**
 * @author Mattia Crispino
 */

import it.unipr.sowide.actodes.interaction.Response;

/**
*
* The {@code Sheperd} enumeration is used for informing the sheperd
* to do something based on which state is sent.

**/

public enum Sheperd implements Response {
	
	/**
	   * Inform the sheperd that it can start moving 
	   * towards the starting point in the map.
	   *
	  **/
	START,
	
	/**
	   * Inform the sheperd to stop it's 
	   * movement.
	   *
	  **/
	STOP;

}

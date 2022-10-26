package interaction;

/**
 * @author Mattia Crispino
 */

import java.awt.Point;
import java.awt.geom.Point2D;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import it.unipr.sowide.actodes.registry.Reference;
import javafx.scene.Node;

/**
*
* The {@code Info} class is used to create message objects that are sent to the boid
* to inform them about either their properties (position, velocity, reference and so on) or 
* about the same properties but for all the other boids.
**/

public class Info implements Serializable {

	private static final long serialVersionUID = 1L;

	private int boidX;
	private int boidY;
	private Point position;
	private Point2D.Double boidVelocity;
	private double boidDirection;
	private Reference refActor;
	private Boolean isSheperd;
	private ArrayList<Node> obstacles;
	private ArrayList<Point> vertexPRM;
	private HashMap<Point, Point> shortestPathPRM;
	private HashMap<Point, ArrayList<Point>> edgesPRM;
 	
	/**
	 * Class constructor.
	 * @param pos	 		the initial position of the boid
	 * @param velocity	    the vector for boid's velocity in x-axis and y-axis
	 * @param direction 	the direction of the boid
	 * @param obs			the list of obstacles found in the map
	 * @param vertex		the list of vertices that compose the PRM (probabilistc roadmap)
	 * @param shortestPath	the map of the shortest path identified in the PRM
	 * @param edges			the map with key = a given vertex and values the list of the vertex to which this is connected to
	 * @param sheperd		flag. If set to true tell that the boid at position "pos" is the sheperd
	 */

	public Info(Point pos, Point2D.Double velocity, double direction, ArrayList<Node> obs, ArrayList<Point> vertex, HashMap<Point, Point> shortestPath, 
			HashMap<Point, ArrayList<Point>> edges, Boolean sheperd) {
		this.position = pos;
		this.boidVelocity = velocity;
		this.boidDirection = direction;
		this.obstacles = obs;
		this.vertexPRM = vertex;
		this.shortestPathPRM = shortestPath;
		this.edgesPRM = edges;
		this.isSheperd = sheperd;
	};
	
	/**
	 * Class constructor with reference to the Actor
	 * @param ref	     the reference of the actor
	 */
	public Info(Point pos, Point2D.Double velocity, double direction, Reference ref, ArrayList<Node> obs, ArrayList<Point> vertex, HashMap<Point, Point> shortestPath, 
			HashMap<Point, ArrayList<Point>> edges, Boolean sheperd) {
		this(pos, velocity, direction, obs, vertex, shortestPath, edges, sheperd);
		this.refActor = ref;
	};
	
	/**
	 * Gets the x coordinate of boid's position.
	 * @return the x coordinate.
	 */
	public int getX() {
		return this.boidX;
	};
	
	/**
	 * Gets the y coordinate of boid's position.
	 * @return the y coordinate.
	 */
	public int getY() {
		return this.boidY;
	}
		
	/**
	 * Gets the reference of the actor.
	 * @return the reference.
	 */
	public Reference getReferenceActor() {
		return this.refActor;
	}
	
	/**
	 * Gets the position (x,y) of the boid
	 * @return the position of the boid
	 */
	public Point getPosition() {
		return this.position;
	}
	
	/**
	 * Get the boid's velocity from the message
	 * @return the point of the velocity
	 */
	public Point2D.Double getVelocity() {
		return this.boidVelocity;
	}
	
	/**
	 * Get the boid's direction 
	 * @return the direction
	 */
	public double getDirection() {
		return this.boidDirection;
	}
	
	/**
	 * Get the list of obstacles in the map
	 * @return the list of obstacles
	 */
	public ArrayList<Node> getObstacles(){
		return this.obstacles;
	}
	
	/** 
	 * Get the vertices that compose the probabilistic roadmap
	 * @return the list of vertices
	 */
	public ArrayList<Point> getVertexPRM(){
		return this.vertexPRM;
	}
	
	/**
	 * Get the shortest path found in the map
	 * @return the map of the shortest path
	 */
	public HashMap<Point, Point> getMapShortestPath(){
		return this.shortestPathPRM;
	}
	
	/**
	 * Get the edges of the roadmap
	 * @return the map with the edges
	 */
	public HashMap<Point, ArrayList<Point>> getEdgesPRM(){
		return this.edgesPRM;
	}
	
	public Boolean getIsSheperd() {
		return this.isSheperd;
	}


	

}

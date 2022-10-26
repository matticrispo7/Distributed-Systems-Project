package flocking;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.io.Serializable;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.ResourceBundle;

import interaction.Continue;
import interaction.Covering;
import interaction.CoveringStop;
import interaction.GoalSearching;
import interaction.Info;
import interaction.Sheperd;
import interaction.SheperdInfo;
import interaction.Update;
import it.unipr.sowide.actodes.actor.Behavior;
import it.unipr.sowide.actodes.actor.CaseFactory;
import it.unipr.sowide.actodes.actor.MessageHandler;
import it.unipr.sowide.actodes.actor.MessagePattern;
import it.unipr.sowide.actodes.actor.Shutdown;
import it.unipr.sowide.actodes.filtering.constraint.IsInstance;
import it.unipr.sowide.actodes.interaction.Kill;
import it.unipr.sowide.actodes.interaction.Stop;
import it.unipr.sowide.actodes.registry.Reference;
import javafx.scene.Node;
import javafx.scene.shape.Circle;
/**
 * This class will define the behaviour that each simulated boid needs to have.
 * @author Mattia Crispino
 */

public class Boid extends Behavior {

	private static final long serialVersionUID = 1L;
	private static final String CONFIG = "resources/configuration";
	private static final int MAP_WIDTH;
	private static final int MAP_HEIGHT;
	private static int MAX_ROUND;
	private static final int DISTANCE;							// distance used to check if a boid is in the neighborhood or not
	private static final int OBSTACLE_DISTANCE;					// value used to compare the distance from a boid to an obstacle
	private static final int SHEPERD_DISTANCE;					// value used to compare the distance from a boid to the sheperd	
	private static final int SENSOR_RANGE;						// boid's sensor range
	private static final int VERTEX_EDGES;						// total number of outgoing edge for each vertex
	private static final Boolean COVERING;						// flag to enable the covering behaviour
	private static final Boolean GOAL_SEARCH;					// flag to enable the goal searching behaviour
	private static final Boolean SHEPERDING;					// flag to enable the sheperding behaviour
	private static final int REPULSIVE_MAG = 50;				// abs(intensity) of the repulsive force
	private static final double COHESION_COEFFICIENT = 100.0;
	private static final double SEPARATION_COEFFICIENT = 5.0;
	private static final int ALIGNMENT_COEFFICIENT = 50;
	private static final MessagePattern INFO 			= MessagePattern.contentPattern(new IsInstance(Info.class));
	private static final MessagePattern STOP 			= MessagePattern.contentPattern(new IsInstance(Stop.class));
	private static final MessagePattern CONTINUE 		= MessagePattern.contentPattern(new IsInstance(Continue.class));
	private static final MessagePattern UPDATE_POSITION = MessagePattern.contentPattern(new IsInstance(Update.class));
	private static final MessagePattern GOAL_SEARCHING  = MessagePattern.contentPattern(new IsInstance(GoalSearching.class));
	private static final MessagePattern COVERING_PATT 	= MessagePattern.contentPattern(new IsInstance(Covering.class));
	private static final MessagePattern COVERING_STOP 	= MessagePattern.contentPattern(new IsInstance(CoveringStop.class));
	private static final MessagePattern SHEPERD_INFO 	= MessagePattern.contentPattern(new IsInstance(SheperdInfo.class));
	private static final MessagePattern SHEPERD_START 	= MessagePattern.contentPattern(new IsInstance(Sheperd.class));
	
	static
	  {
	    ResourceBundle b = ResourceBundle.getBundle(CONFIG);
	    MAP_WIDTH = Integer.parseInt(b.getString("map.width"));
	    MAP_HEIGHT = Integer.parseInt(b.getString("map.height"));
	    DISTANCE = Integer.parseInt(b.getString("neighborhood.distance"));
	    OBSTACLE_DISTANCE = Integer.parseInt(b.getString("boid.obstacle.distance"));
	    SENSOR_RANGE = Integer.parseInt(b.getString("sensor.range"));
	    VERTEX_EDGES = Integer.parseInt(b.getString("vertex.edges"));
	    MAX_ROUND = Integer.parseInt(b.getString("round"));
	    COVERING = Boolean.parseBoolean(b.getString("covering"));
	    GOAL_SEARCH = Boolean.parseBoolean(b.getString("goal.searching"));
	    SHEPERDING = Boolean.parseBoolean(b.getString("sheperding"));
	    SHEPERD_DISTANCE = Integer.parseInt(b.getString("sheperd.distance"));
	  }
	
	private HashMap<Reference, Boid> 			boidMap;					// map used to keep track of other boids info =>  key = boid's reference 	value = boid's object
	private ArrayList<Node> 					obstacles;					// list of obstacles found in the map
	private ArrayList<Point> 					vertexPRM;					// list of vertices that are included in the PRM (in DESCENDING order based on the point's weight)
	private ArrayList<Point> 					positionCovered;			// array that stores all the position that this boid has covered (for the COVERING behaviour)
	private ArrayList<Point> 					nodeVisited;				// list of the nodes (vertex of the PRM) visited by the boid (used for COVERING behaviour)
	private ArrayList<Point>  					vertexAlreadyChecked;		// list of the vertices of the PRM that have already been checked (for all the outgoing edges) by the boid (used for COVERING)
	private HashMap<Point, Integer> 			verticesWeights;			// map used to store the weights for every vertex in the PRM
	private HashMap<ArrayList<Point>, Integer>  edgesWeightsPRM;			// key = list of point (sorted by increasing order) connected by the edge   value = edge's weight (initial set to 1) (usde for COVERING behaviour)
	private HashMap<Point, ArrayList<Point>> 	edgesPRM;					// map with key = point and values = to all the other points that this point is connected with an edge 
	private HashMap<Point, Point> 				shortestPathPRM;			// map used to store the shortest path found in the PRM
	private HashMap<ArrayList<Point>, Boolean> 	edgesAlreadyFollowed;		// key = list of point (sorted by increasing order) connected by the edge  value = true (if the edge has already be followed)
	
	private Integer currentVertexWeight;									// log of the current subgoal's weight to which the boid is directed
	private Point finalGoal;												// goal position where the boid needs to go
	private Point currentSubgoal;											// boid's current subgoal
	private Reference initiatorReference;									// reference to the initiator	
	private int N_ROUND;													// usata per log per vedere i comportamenti dei boid in N round (viene incrementata in moveBoid() )
	private int x;	
	private int y;
	private double direction;												// boid's direction
	//private int sensorRange;												// boid's sensor range
	private boolean goalSearchingFound;										// flag that if set to 1 tells the boid to be attracted by the final Goal (for the "goal searching" behaviour)
	private boolean coveringEnabled;										// flag that if set to true tells the boid to use the "covering" behaviour
	private boolean goalSerchingEnabled;									// flag that if set to true tells the boid to use the "goal searching" behaviour
	private boolean sheperdingEnabled;										// flag that if set to true tells the boid to use the "sheperding" behaviour
	private boolean sheperdStart;											// flag that if set to true allows the sheperd to move and start it's behaviour
	private Point position;													// boid's position
	private Point2D.Double velocity;										// boid's velocit
	private Point sheperdPosition;											// sheperd's position
	private Point2D.Double sheperdVelocity;									// sheperd's velocity
	private Reference sheperdReference;										// sheperd's reference
	private int updateMsgReceived;											/* used as counter to keep track of how many update msg are received (if it is = to the length of the map in which each boid
																			store the info about every other boid then this boid can go "to the next step" and calculate its new position
																			*/
	
	public Boid(Point pos, Point2D.Double velocity, double direction) {
		this.position 					= pos;
		this.velocity 					= velocity;
		this.direction 					= direction;
		this.updateMsgReceived 			= 0;		
		this.N_ROUND 					= 0;				
		this.currentVertexWeight 		= 0;
		this.initiatorReference 		= null;
		this.obstacles 					= new ArrayList<>();
		this.vertexPRM 					= new ArrayList<>();
		this.nodeVisited 				= new ArrayList<>();
		this.vertexAlreadyChecked 		= new ArrayList<>();
		this.positionCovered 			= new ArrayList<>();
		this.boidMap 					= new HashMap<>();
		this.verticesWeights 			= new HashMap<>();
		this.shortestPathPRM 			= new HashMap<>();
		this.edgesWeightsPRM 			= new HashMap<>();
		this.edgesPRM 		 			= new HashMap<>();
		this.edgesAlreadyFollowed 		= new HashMap<>();
		this.finalGoal 					= new Point(0,0);
		this.currentSubgoal  			= new Point(0,0);
		this.sheperdPosition 			= new Point(0,0);
		this.sheperdVelocity			= new Point2D.Double();
		this.sheperdReference			= null;
		this.goalSearchingFound 		= false;
		this.sheperdStart				= false;
		this.coveringEnabled 			= COVERING;
		this.goalSerchingEnabled 		= GOAL_SEARCH;
		this.sheperdingEnabled 			= SHEPERDING;
	}
	

	@Override
	public void cases(CaseFactory c) {
		
		c.define(INFO, boidsPosition());
		c.define(SHEPERD_INFO, sheperdInfoHandler());
		
		MessageHandler kill = (m) -> {
			return Shutdown.SHUTDOWN;
		};
		c.define(KILL, kill);

		// pattern-handler that catch the "continue" msg sent by the initiator
		MessageHandler cont = (m) -> {
			moveBoid();
			return null;
		};
		c.define(CONTINUE, cont);
		
		c.define(GOAL_SEARCHING, goalSearchingHandler());
		
		c.define(COVERING_PATT, coveringHandler());
		
		// update the position of the boid according to the position of its neighborhood
		c.define(UPDATE_POSITION, updatePosition());
		
		c.define(SHEPERD_START, sheperdStartHandler());
		
		// start code
		MessageHandler h = (m) -> {
			System.out.println(getReference() + " Boid created at " + this.getBoidPosition() + " with velocity " + this.getVelocity() + " and direction " + this.getDirection());
			return null;
		};
		c.define(START, h);
		
		c.define(ACCEPTALL, DUMMY);

	}

	/*
	 * This method will save the info about this boid sent by the initiator
	 */
	private MessageHandler boidsPosition() {
		return (m) -> {
			// save the initiator reference
			if(this.getInitiatorReference() == null) {
				this.setInitiatorReference(m.getSender());
			}
			Info p = (Info) m.getContent();
			
			// save the obstacles position,the vertices of the PRM, the map of the shortest path and the edges of the PRM
			this.setObstacles(p.getObstacles());
			this.setVertexPRM(p.getVertexPRM());
			this.setShortestPath(p.getMapShortestPath());
			this.setEdgesPRM(p.getEdgesPRM());
			
			// save the final goal position (it is the vertex with the MAX weight=
			this.setFinalGoal(p.getVertexPRM().get(0));
			
			// set the weights (in decreasing order) for the vertex and store them in the map
			int weight = p.getVertexPRM().size();		// set the initial weight as the MAX value
			for (Point v : p.getVertexPRM()) {
				this.verticesWeights.put(v, weight);
				weight-=1;
			}
			
			// initalize all the edges to have weight = 1 (for COVERING behaviour)
			for (var entry : this.getEdgesPRM().entrySet()) {
				Point start = entry.getKey();
				for (Point dest : entry.getValue()) {
					ArrayList<Point> tmp = new ArrayList<>();
					if(this.verticesWeights.get(start) > this.verticesWeights.get(dest)) {
						tmp.add(dest);
						tmp.add(start); 
					} else {
						tmp.add(start);
						tmp.add(dest);
					}// set the weight in the map equal to 1
					this.edgesWeightsPRM.put(tmp, 1);
				}
			}
		
			// set the current subgoal for the boid choosing the one that has the min distance
			int minDistance = Integer.MAX_VALUE;
			for (Point subgoal : p.getVertexPRM()) {
				if(this.getBoidPosition().distance(subgoal) < minDistance) {
					minDistance = (int) this.getBoidPosition().distance(subgoal);
					this.setCurrentVertexWeight(this.verticesWeights.get(subgoal));
					this.setCurrentSubgoal(subgoal);
				}
			}

			if (p.getReferenceActor() == getReference()) {
				this.setBoidPosition(p.getPosition());
			} else {
				boidMap.put(p.getReferenceActor(), new Boid(p.getPosition(), p.getVelocity(), p.getDirection()));
			}

			return null;
		};
	}
	
	/*
	 * This is the handler for the goal searching behaviour
	 */
	private MessageHandler goalSearchingHandler() {
		return (m) -> {
			GoalSearching msg = (GoalSearching) m.getContent();
			// update the goal position of this boid setting the position received by the message
			this.setFinalGoal(msg.getPosition());
			// inform the boid to be attracted by that goal
			this.goalSearchingFound = true;
			return null;
		};
	}
	
	/*
	 * The handler associated to the Update message
	 */
	private MessageHandler updatePosition() {
		return (m) -> {
			Update msg = (Update) m.getContent();
			// check if the sender is different from this boid
			if(msg.getSenderReference() != getReference())
			{
				// update the boid's position in the map
				boidMap.put(msg.getSenderReference(), new Boid(msg.getPosition(), msg.getVelocity(), msg.getDirection()));
				// keep track of how many update messages have been received;
				this.updateMsgReceived += 1;
			}
			return null;
		};
	}
	
	private void moveBoid() {
		System.out.println();
		/* Apply cohesion, alignment and separation to the boid */
		Point cohesionCoordinates = cohesion();
		Point separationCoordinates = separation();
		List<Serializable> alignmentInfo = alignment();
		Point2D.Double alignment = (Double) alignmentInfo.get(0);
		this.setDirection((double) alignmentInfo.get(1));
		Point repForce = repulsiveForce();
		Point atrForce = new Point(0,0);
		
		if(this.goalSerchingEnabled) {
			// apply the GOAL SEARCHING behaviour 
			goalSearching();
		}
		
		// differentiate the boid's movement based on which flag is enables
		if(this.goalSearchingFound) { 	
			atrForce = null;
			if(this.getBoidPosition().distance(this.getFinalGoal()) < SENSOR_RANGE) {
				atrForce = attractionForce(this.getFinalGoal(), 10);	// attraction force based on the position received by the GOAL SEARCHING
			} else {
				atrForce = attractionForce(this.getFinalGoal(), 5);	// attraction force based on the position received by the GOAL SEARCHING
			}
		} else if (this.coveringEnabled){			
			// the current subgoal is set as the closest vertex in the map (in the "boidsPosition()" method)
			/* In first instance, if the current subgoal is not in boid's sensor range then apply an attraction force towars the current subgoal.
			 * When the current subgoal is in the boid's sensor range, call the covering() method such that it will find the closest vertex (that is equal to the current subgoal)
			 * and then it will select the outgoing edge (so the node to which the boid is redirected) with the lowest weight.
			 * At the end, set the destination node returned by the covering() as the current subgoal (so that the boid, at next iteration, will trigger the if condition otherwise
			 * it will select another node and so on)
			 */
			if(this.getBoidPosition().distance(this.getCurrentSubgoal()) > SENSOR_RANGE) {
				// apply the attraction force
				atrForce = attractionForce(this.getCurrentSubgoal(), 5);
				System.out.println("===> Boid " + this.getReference() + " is attracted and goind towards " + this.verticesWeights.get(this.getCurrentSubgoal()));
			} else {
				Point destinationNode = covering();	// check if there's a PRM vertex inside the boid's sensor range
				if(destinationNode != null) {
					this.setCurrentSubgoal(destinationNode);
					atrForce = attractionForce(destinationNode, 5);		// if a node is found, the attraction force is based on the position received by the COVERING (so by the closest PRM vertex)
					/*System.out.println("===> Boid " + this.getReference() + " current subgoal " + this.verticesWeights.get(this.getCurrentSubgoal()));
					System.out.println("===> Boid " + this.getReference() + " is going towards " + this.verticesWeights.get(destinationNode));
					System.out.println("\n\n\nEdges weights");
					for (var entry : this.edgesWeightsPRM.entrySet()) {
						System.out.println("Edge " + this.verticesWeights.get(entry.getKey().get(0)) + "->" + this.verticesWeights.get(entry.getKey().get(1)) + " weight=" + entry.getValue());
					}
					System.out.println("\n\n\n");*/
				} else {
					// the boid has visited all the nodes in the map.  
					send(initiatorReference, CoveringStop.COVERING_STOP);
				}
				
			}			
		} else if(this.sheperdingEnabled) {
			if(this.getReference() == this.getSheperdReference() && !this.getSheperdStart()) { // while all the boid are not at the 1st vertex, keep the sheperd still
				this.N_ROUND += 1;
				send(SPACE, new Update(this.getReference(), this.getBoidPosition(), this.getVelocity(), this.getDirection(), this.getRound(), null));
				return;
			}
	
			// BOID => check the distance from the sheperd
			if(this.getReference() != this.getSheperdReference() && (this.getBoidPosition().distance(boidMap.get(this.getSheperdReference()).getBoidPosition()) < SHEPERD_DISTANCE)) {
				if(this.getBoidPosition().distance(this.getCurrentSubgoal()) < SENSOR_RANGE) { // next subgoal
					if(this.getCurrentSubgoal() != this.getFinalGoal()) {
						this.setCurrentSubgoal(this.getShortestPath().get(this.getCurrentSubgoal()));
					} else {
						this.setCurrentSubgoal(this.getFinalGoal());
					}
				}
				//System.out.println("===> Boid " + this.getReference() + " is going towards " + this.verticesWeights.get(this.getCurrentSubgoal()));
				repForce = repulsiveForce(this.boidMap.get(this.getSheperdReference()).getBoidPosition(), 5); // REPULSIVE FORCE FROM THE SHEPERD
				// attract a bit to the next subgoal
			} else if(this.getReference() == this.getSheperdReference() && this.getSheperdStart()) { // SHEPERD
				// if the sheperd is at the current subgoal and there are no boids left behind => set the next subgoal on the map
				if(!boidLeftBehind() && (this.getBoidPosition().distance(this.getCurrentSubgoal()) < SENSOR_RANGE)) {
					if(this.getCurrentSubgoal() != this.getFinalGoal()) {
						this.setCurrentSubgoal(this.getShortestPath().get(this.getCurrentSubgoal()));
					}  else {
						this.setCurrentSubgoal(this.getFinalGoal());
					}
				}
			}
			if(this.getReference() == this.getSheperdReference()) {
				atrForce = attractionForce(this.getCurrentSubgoal(), 15);
			} else {
				atrForce = attractionForce(this.getCurrentSubgoal(), 10); 
			}
			
			// ------------------------ SHEPERD END  ------------------------ 
		}  else {
			atrForce = attractionForce();	// the boid is attracted by the first vertex of the list (this is identified as the goal)
		}

		// bound the boid's position inside the map
		Point boundPos = boundPosition(50);

		/*  =========================
		 		BASIC UPDATE RULE: 
	 		=========================
		 Velocity ===== > b.velocity = b.velocity + v1 + v2 + v3 + ... + vn
		 Position ===== > b.position = b.position + b.velocity
		 
		 If the "covering" or the "sheperding" is selected, then the rule will be slightly different.
		 */
		double vx, vy;
		if(this.coveringEnabled) {
			vx = this.getVelocity().getX() + boundPos.getX() + repForce.getX() + atrForce.getX();
			vy = this.getVelocity().getY() + boundPos.getY() + repForce.getY() + atrForce.getY();
		} else if(this.sheperdingEnabled) {
			if(this.getReference() != this.getSheperdReference()) {
				if(this.getSheperdStart()) {
					vx = separationCoordinates.getX() + repForce.getX() + boundPos.getX() + atrForce.getX();
					vy = separationCoordinates.getY() + repForce.getY() + boundPos.getY() + atrForce.getY();
				} else {
					vx = cohesionCoordinates.getX() + separationCoordinates.getX() + alignment.getX() + repForce.getX() + atrForce.getX() + boundPos.getX();
					vy = cohesionCoordinates.getY() + separationCoordinates.getY() + alignment.getY() + repForce.getY() + atrForce.getY() + boundPos.getY();
				}
				
			} else {
				vx = repForce.getX() + atrForce.getX() + boundPos.getX();
				vy = repForce.getY() + atrForce.getY() + boundPos.getY();
			}
			
		} else {
			vx = this.getVelocity().getX() + cohesionCoordinates.getX() + separationCoordinates.getX() + alignment.getX() + repForce.getX() + atrForce.getX() + boundPos.getX();
			vy = this.getVelocity().getY() + cohesionCoordinates.getY() + separationCoordinates.getY() + alignment.getY() + repForce.getY() + atrForce.getY() + boundPos.getY();
		}

		this.setVelocity(vx, vy);
		limitVelocity(this, 20);
		// get new position of the boid
		int boidX = (int) (this.getBoidPosition().getX() + this.getVelocity().getX());
		int boidY = (int) (this.getBoidPosition().getY() + this.getVelocity().getY());
			
		// set the new position of the boid
		this.setBoidPosition(new Point(boidX, boidY));
		
		// save the position in the positionCovered if the "COVERING" behaviour is chosen
		if(this.coveringEnabled) {
			this.positionCovered.add(new Point(boidX, boidY));
		}
		
		System.out.println("[ROUND: " + this.N_ROUND + "] " + getReference() + " position: " + boidX + ","+boidY + " velocity: " + this.getVelocity().getX() + "," + this.getVelocity().getY());
		
		// keep track of the number of round for each boid
		this.N_ROUND += 1;
		send(SPACE, new Update(this.getReference(), this.getBoidPosition(), this.getVelocity(), this.getDirection(), this.getRound(), null));
		
	}

	/*
	 * Apply the cohesion behaviour
	 * @return the Point computed by this behaviour
	 */
	private Point cohesion() {
		// iterate all over the boids and find the ones in the neighborhood
		int neighborhoodCount = 0;
		// goal position starts in (0,0)
		Point goal = new Point(0, 0);
		for (var entry : boidMap.entrySet()) {
			double distance = distanceBetweenPoints(this.getBoidPosition(), entry.getValue().getBoidPosition());
			if (distance < DISTANCE) {
				neighborhoodCount += 1;
				goal.x += entry.getValue().getBoidPosition().getX();
				goal.y += entry.getValue().getBoidPosition().getY();
			} 
		}
		// if no neighbor are found then the goal position is set to the actual position of the boid
		if (neighborhoodCount != 0) {
			// normalize the position
			goal.x /= neighborhoodCount;
			goal.y /= neighborhoodCount;
		} else {
		}
		
		// limit the cohesion such that the boid will move by a factor (COHESION_COEFFICIENT) towards the centre of mass of the neighbor flock
		Point tmp = new Point();
		tmp.x = (int) ((goal.getX() - this.getBoidPosition().getX())/COHESION_COEFFICIENT);
		tmp.y = (int) ((goal.getY() - this.getBoidPosition().getY())/COHESION_COEFFICIENT);
		return tmp;
	};

	/*
	 * Apply the separation behaviour
	 * @return the Point computed by this behaviour
	 */
	private Point separation() {
		// iterate all over the boids and find the ones in the neighborhood
		int neighborhoodCount = 0;
		// goal position starts in (0,0)
		Point goal = new Point(0,0);
		for (var entry : boidMap.entrySet()) {
			double diffx = entry.getValue().getBoidPosition().getX() - this.getBoidPosition().getX();
			double diffy = entry.getValue().getBoidPosition().getY() - this.getBoidPosition().getY();
			double magnitude = Math.sqrt((Math.pow(diffx, 2)) + (Math.pow(diffy, 2)));
			if (magnitude < SEPARATION_COEFFICIENT) {
				neighborhoodCount += 1;
				goal.x -= (entry.getValue().getBoidPosition().getX() - this.getBoidPosition().getX());
				goal.y -= (entry.getValue().getBoidPosition().getY() - this.getBoidPosition().getY());

			} 
		}
		return goal;
	};

	/*
	 * Apply the alignment behaviour with respect to the other boids
	 * @return 		an array which contains, in position 0, the Point2D of the velocity and in position 1 the direction
	 */
	private List<Serializable> alignment() {
		int neighborhoodCount = 0;
		Point2D.Double velocity = new Point2D.Double(0,0);
		double direction = 0;
		if(!this.sheperdingEnabled) {
			for(var entry : boidMap.entrySet()) {
				double distance = distanceBetweenPoints(this.getBoidPosition(), entry.getValue().getBoidPosition());
				if(distance < DISTANCE) {
					neighborhoodCount += 1;
					// sum the boid's velocity and the direction
					velocity.x += entry.getValue().getVelocity().getX();
					velocity.y += entry.getValue().getVelocity().getY();
					direction += entry.getValue().getDirection();
				}
			}
		} else {
			for(var entry : boidMap.entrySet()) {
				if(entry.getKey() != this.getSheperdReference()) {
					double distance = distanceBetweenPoints(this.getBoidPosition(), entry.getValue().getBoidPosition());
					if(distance < DISTANCE) {
						neighborhoodCount += 1;
						// sum the boid's velocity and the direction
						velocity.x += entry.getValue().getVelocity().getX();
						velocity.y += entry.getValue().getVelocity().getY();
						direction += entry.getValue().getDirection();
					}
				}
			}
		}
		
		// if there are no neighbors => return the velocity
		if(neighborhoodCount == 0) {
			return Arrays.asList(this.getVelocity(), direction);
		} 
		
		// otherwise divide the velocity by the #neighbors found and then normalize
		velocity.x /= neighborhoodCount;
		velocity.y /= neighborhoodCount;
		direction /= neighborhoodCount;
		
		Point2D.Double tmp = new Point2D.Double(0,0);
		tmp.x = (velocity.getX() - this.getVelocity().getX()) / ALIGNMENT_COEFFICIENT;
		tmp.y = (velocity.getY() - this.getVelocity().getY()) / ALIGNMENT_COEFFICIENT;
		return Arrays.asList(tmp, direction);
	};
	
	/**
	 * In order to keep the flock within a certain area, implement a rule which encourages them to stay within rough boundaries;
	 * @return   the next point which is closer to the boundaries of the map
	 */
	public Point boundPosition(int offset) {
		int Xmax = MAP_WIDTH, Ymax = MAP_HEIGHT;
		Point goal = new Point(0,0);

		if(this.getBoidPosition().getX() < 0) {
			goal.x = offset;
		} else if (this.getBoidPosition().getX() > Xmax){
			goal.x = -offset;
		} 
		if(this.getBoidPosition().getY() < 0) {
			goal.y = offset;
		} else if(this.getBoidPosition().getY() > Ymax) {
			goal.y = -offset;
		}
		return goal;
	}

	/*
	 * The @param goal is the place that the boid is attracted to.
	 */
	private Point attractionForce(Point goal, int coefficient) {
		Point tmp = new Point();
		tmp.x = (int) ((goal.x - this.getBoidPosition().getX())/coefficient);
		tmp.y = (int) ((goal.y - this.getBoidPosition().getY())/coefficient);
		return tmp;
	}
	
	/* if the goal is in the view range of the boid 			  => stay around the goal
	 * else if current subgoal is in the view range of the boid   => set next subgoal as the target
	 * Each subgoal is identified by a weight (the greater the weight, the nearer is the subgoal to the goal) so (ideally) each boid will
	 * move from a subgoal to one with a greater weight since it will reach the final goal.
	 */
	private Point attractionForce() {
		Point tmp = new Point();
		if(this.getBoidPosition().distance(this.getFinalGoal()) < SENSOR_RANGE) {
			System.out.println("\n\n\n\nBoid " + this.getReference() + " has the goal " + this.getFinalGoal().toString() + " in its view range with distance " + this.getBoidPosition().distance(this.getFinalGoal()) + "\n\n\n\n"); 
			tmp.x = (int) ((this.getFinalGoal().getX() - this.getBoidPosition().getX())/5);	
			tmp.y = (int) ((this.getFinalGoal().getY() - this.getBoidPosition().getY())/5);
			this.setCurrentSubgoal(this.getFinalGoal());
		} else if(this.getBoidPosition().distance(this.getCurrentSubgoal()) < SENSOR_RANGE) { // set the next subgoal
			System.out.println("Boid " + this.getReference() + " has the SUBGOAL " + this.getCurrentSubgoal().toString() + " in its view range with distance " + this.getBoidPosition().distance(this.getCurrentSubgoal())); 
			
			// get the next subgoal which has the min distance from this current subgoal (from the shortest path defined)
			Point nextSubgoal = this.getShortestPath().get(this.currentSubgoal);
			this.setCurrentSubgoal(nextSubgoal);
			
			tmp.x = (int) ((this.getCurrentSubgoal().getX() - this.getBoidPosition().getX())/10);
			tmp.y = (int) ((this.getCurrentSubgoal().getY() - this.getBoidPosition().getY())/10);
		} else { 
			// attract a little bit the boid to its current subgoal
			tmp.x = (int) ((this.getCurrentSubgoal().getX() - this.getBoidPosition().getX())/50);
			tmp.y = (int) ((this.getCurrentSubgoal().getY() - this.getBoidPosition().getY())/50);
		} 
		
		return tmp;
	}
	
	/* In order to implement the goal searching behaviour, each boid can move based on the 3 main behaviour (cohesion, separation and alignment) and with the repulsive force of the obstacles.
	 * Each boid has the info about where the goal is located but they don't use it (so they don't exploit the goal in order to have an attraction force linked to it) so every boid basically
	 * is "exploring" the enviroment.
	 * When the first boid, covering the enviroment, senses that the goal is in its sensor range, it sends its position (more or less its position is equal to the goal) to all the other boids
	 * such that they can set that position as their next "subgoal" and then have an attraction force to it. 
	 */
	private void goalSearching() {
		// check if the goal is in the sensor range of the boid
		if(this.getBoidPosition().distance(this.getFinalGoal()) < SENSOR_RANGE) {
			// the boid has found the goal => send the msg to the other boids
			System.out.println("Boid " + this.getReference() + " has found the goal in position " + this.getBoidPosition());
			send(SPACE, new GoalSearching(this.getReference(), this.getBoidPosition(), this.getVelocity()));
		}
	}
	
	/*
	 * This handler allows a boid (if it's reference is different from the sender) to update the weight of the edge that are attached in the message sent
	 */
	private MessageHandler coveringHandler() {
		return (m) -> {
			Covering msg = (Covering) m.getContent();
			if(msg.getSenderReference() != this.getReference()) {
				// increase the weight of the edge received
				this.increaseEdgeWeightPRM(msg.getEdge());
			}
			
			return null;
		};
	}
	
	/*
	 * This handler allows both the boids and the sheperd to save the properties of the sheperd such velocity, position and reference
	 */
	private MessageHandler sheperdInfoHandler() {
		return (m) -> {
			SheperdInfo msg = (SheperdInfo) m.getContent();
			if(msg.getSheperdRef() != this.getReference()) {
				// save the info about the sheperd
				boidMap.put(msg.getSheperdRef(), new Boid(msg.getSheperdPos(), msg.getSheperVel(), 0));
				this.setSheperdReference(msg.getSheperdRef());
			} else {
				// set the properties of the sheperd
				this.setVelocity(msg.getSheperVel());
				this.setBoidPosition(msg.getSheperdPos());
				this.setSheperdReference(msg.getSheperdRef());
			}
			// set the current subgoal as the first vertex of the PRM
			this.setCurrentSubgoal(this.vertexPRM.get(this.vertexPRM.size()-1));
			
			return null;
		};
	}
	
	/*
	 * Once either a boid or the sheperd receive a Sheperd.START message, they will update the flag associated to the sheperd's movement.
	 */
	private MessageHandler sheperdStartHandler() {
		return (m) -> {
			this.setSheperdStart(true);
			return null;
		};
	}
	
	/* Loop through every vertex of the PRM, (1) find the vertex that hasn't already been checked and which has the min.distance from boid's position and (2) check if that distance is < SENSOR_RANGE.
	 * If yes, find the outgoind edge (starting from the closest node) with lowest weight, increment the edge's weight and return the destination vertex (the node to
	 * which the boid needs to be attracted)
	 * Otherwise return the closest node such that the boid, applying the attraction force, will move closer to it
	 */
	private Point covering() {
		// FIND THE CLOSEST VERTEX IN THE MAP
		double minDistance = Integer.MAX_VALUE;
		Point closestVertex = new Point();
		if(this.getVertexChecked().size() != VERTEX_EDGES) {  // find the destination node if this boid hasn't visited all the nodes
			for (Point vertex : this.getVertexPRM()) {
				//System.out.println("Boid " + this.getReference() + " checking point " + vertex);
				if(!this.getVertexChecked().contains(vertex)) {
					if(this.getBoidPosition().distance(vertex) < minDistance) {    // (1)
						minDistance = this.getBoidPosition().distance(vertex);	
						closestVertex = vertex;
					}
				} 
			}
		} else { // the boid has visited all the vertices in the map
			return null;
		}
		if(closestVertex == null) { // the boid has visited all the vertices in the map
			return null;
		} else {
			if(minDistance < SENSOR_RANGE) {
				/* 	Get the edge with lowest weight and check if the destination node hasn't already been visited by the boid
					=> if all of the edges have the same weight, choose one randomly (and check if hasn't been visited)
					=> if there's not an unvisited destination node (for the selected edges), find the 2nd closest vertex in the map and compute again all the checks
				*/
				
				// FIND THE EDGE TO FOLLOW
				System.out.println("Boid " + this.getReference() + " reached the destination " + this.verticesWeights.get(closestVertex));
				//while(true) { 
					int minEdgeWeight = Integer.MAX_VALUE;
					int nodesAlreadyVisited = 0;
					int edgeFollowed = 0;
					ArrayList<Point> edgeToSend = new ArrayList<>();
					Point destinationNode = new Point();
					for (Point destination : this.getEdgesPRM().get(closestVertex)) {   // get the edges for the closest node (so get the nodes connected to this "closest")
						// check if the node hasn't already been visited by the boid
							//System.out.println("Destination node " + destination + " hasn't already been visited by the Boid " + this.getReference());
						ArrayList<Point> mapKey = new ArrayList<>();	// sort by increasing weight the list [closestVertex, destinationVertex] and get from the map the edge's weight
						if(this.verticesWeights.get(closestVertex) > this.verticesWeights.get(destination)) {
							mapKey.add(destination);
							mapKey.add(closestVertex);
						} else {
							mapKey.add(closestVertex);
							mapKey.add(destination);
						}
						// first check if this edge has already been followed by the boid (if it's null it means that the edge hasn't been followed yet)
						// get the edge's weight from the map
						//System.out.println("[LOG] Edge " + this.verticesWeights.get(mapKey.get(0)) + " -> " + this.verticesWeights.get(mapKey.get(1)) + " weight: " + this.getEdgesWeightPRM().get(mapKey));
						if(this.getEdgesWeightPRM().get(mapKey) < minEdgeWeight) {
							minEdgeWeight = this.getEdgesWeightPRM().get(mapKey);
							destinationNode = destination;
							edgeToSend = mapKey;
						}
					}
					int initialEdgeWeight = this.edgesWeightsPRM.get(edgeToSend);
					this.edgesWeightsPRM.put(edgeToSend, (initialEdgeWeight+1));   // increase the weight of the edge chosen
					// add the destination node as one visited by this boid
					if(!this.nodeVisited.contains(destinationNode)) {
						this.nodeVisited.add(destinationNode);
						send(SPACE, new Covering(edgeToSend, this.getReference()));
						return destinationNode;
					} else {
						// check if the boid has visited all the nodes in the map
						if(this.nodeVisited.size() == this.vertexPRM.size()) {
							//System.out.println("\n\n\nALL THE NODES VISITED\n\n\n");
							return null;
						} else {
							send(SPACE, new Covering(edgeToSend, this.getReference()));
							return destinationNode;
						}
					}
				
			} else {
				// return the closest node such that the boid will apply the attraction force
				return closestVertex;
			}
		}
	}
	
	/*
	 * This method allows to check if there are some boids left behind in the path.
	 * The sheperd will go back to the first node of the path (the vertex with lowest weight) and check if there are some boids close to it.
	 * This is done for every node in the shortest path until the actual node checked is equal to the current subgoal of the sheperd.
	 * @return true if there are some boids left behind, false otherwise
	 */
	private Boolean boidLeftBehind() {
		// loop through every previous subgoal
		Point subgoalChecked = this.getVertexPRM().get(this.getVertexPRM().size()-1);   // the subgoal that the sheperd is checking if there are some boids near to it left behind (start from the first vertex)
		while(subgoalChecked != this.getCurrentSubgoal()) {
			int boidLeftBehind = 0;
			// check if there are some boids left behind (these are the fartest from the ACTUAL SUBGOAL and closer to a previous subgoal)
			for (var entry : this.boidMap.entrySet()) {
				if(entry.getKey() != this.getReference()) { // do not perform this check for the sheperd
					if(entry.getValue().getBoidPosition().distance(subgoalChecked) <= SENSOR_RANGE) {
						boidLeftBehind += 1;
					}
				}
			}
			if(boidLeftBehind != 0) {  // there are some boid close to a previous subgoal
				// the sheper will be attracted to that subgoal and catch up these boids
				this.setCurrentSubgoal(subgoalChecked);
				return true;
			}
			subgoalChecked = this.getShortestPath().get(subgoalChecked);
		}
		return false;
	}	
	
	/*
	 * This method will apply a repulsive force (for the boid that has invoked the method) from all the obstacles in the map if these have a distance that is
	 * less the property OBSTACLE_DISTANCE.
	 * The value property REPULSIVE_MAG is associated to the magnitude of the repulsive force so the greater this value further way the boid will be
	 * @return the point to which the boid needs to be redirected
	 */
	private Point repulsiveForce() {
		Point goal = new Point(0,0);
		Point tmp = new Point();
		// loop for every obstacles, find the distance and select the obstacle with smaller distance
		// if the distance < VALUE then apply the repulsive force
		for (Node node : this.getObstacles()) {
			// check the shape of the obstacle
			if(node instanceof Circle) {
				Circle obs = (Circle) node;
				// find the distance between this boid position and the center of the obstacle
				Point boidPos = new Point((int) this.getBoidPosition().getX(), (int) this.getBoidPosition().getY());
				Point obsPos  = new Point((int) obs.getCenterX(), (int) obs.getCenterY());
				if((distanceBetweenPoints(boidPos, obsPos) - obs.getRadius()) < OBSTACLE_DISTANCE) {
					goal.x = (int) obs.getCenterX();
					goal.y = (int) obs.getCenterY();
					// set the returning value 
					tmp.x = (int) (-REPULSIVE_MAG * (goal.getX() - this.getBoidPosition().getX())/50);
					tmp.y = (int) (-REPULSIVE_MAG * (goal.getY() - this.getBoidPosition().getY())/50);					
				}
			}
		}
		
		return tmp;
	}
	
	/*
	 * This method will apply a repulsive force from a specified point (the @param pos) and the effect of this force can be customized with the @param coefficient
	 * @return the point to which the boid needs to be redirected
	 */
	private Point repulsiveForce(Point pos, int coefficient) {
		Point tmp = new Point();
		if(this.getBoidPosition().distance(pos) < OBSTACLE_DISTANCE) {
			// set the returning value
			tmp.x = (int) (-REPULSIVE_MAG * (pos.getX() - this.getBoidPosition().getX())/coefficient);
			tmp.y = (int) (-REPULSIVE_MAG * (pos.getY() - this.getBoidPosition().getY())/coefficient);
		}
		return tmp;
	}
	
	/**
	 * Get the boid's actual position in the map 
	 * @return  the point of the boid's position
	 */
	public Point getBoidPosition() {
		return this.position;
	}
	
	/**
	 * Get the direction of the map
	 * @return the direction value
	 */
	public double getDirection() {
		return this.direction;
	}
	
	/**
	 * Get the actual velocity of the boid 
	 * @return the point which stores the value of the velocity along both axis
	 */
	public Point2D.Double getVelocity() {
		return this.velocity;
	}
	
	/**
	 * Set the velocity of the boid
	 * @param v  the point which stores the value of the velocity along both axis
	 */
	public void setVelocity(Point2D v) {
		this.velocity.x = v.getX();
		this.velocity.y = v.getY();
	}
	
	
	public void setVelocity(double vx, double vy) {
		this.velocity.x = vx;
		this.velocity.y = vy;
	}
	
	/**
	 * Increase the velocity of a boid
	 * @param v the point which values need to be added to the boid velocity
	 */
	public void boidAddVelocity(Point2D v) {
		this.velocity.x += v.getX();
		this.velocity.y += v.getY();
	}
	
	/**
	 * Set the position of the boid in the map
	 * @param p  the point where the boid needs to be
	 */
	public void setBoidPosition(Point p) {
		this.position = p;
	}
	
	/**
	 * Set the direction of the boid
	 * @param d  the value of the direction
	 */
	public void setDirection(double d) {
		this.direction = d;
	}
	
	/**
	 * Get the obstacles that are in the map
	 * @return  the list of the obstacles
	 */
	public ArrayList<Node> getObstacles(){
		return this.obstacles;
	}
	
	/**
	 * Set the actual obstacles that are in the map
	 * @param obs  the list of the obstacles
	 */
	public void setObstacles(ArrayList<Node> obs) {
		this.obstacles = obs;
	}
	
	/** 
	 * Store the vertices of the probabilist roadmap
	 * @param v  the list of vertices
	 */
	public void setVertexPRM(ArrayList<Point> v) {
		this.vertexPRM = v;
	}
	
	/**
	 * Get all the vertices that compose the roadmap
	 * @return the list of vertices of the PRM
	 */
	public ArrayList<Point> getVertexPRM(){
		return this.vertexPRM;
	}
	
	/** 
	 * Get the acutal round of execution
	 * @return the round
	 */
	public int getRound() {
		return this.N_ROUND;
	}
	
	/**
	 * Set the reference of the initiator
	 * @param ref  the initiator's reference
	 */
	public void setInitiatorReference(Reference ref) {
		this.initiatorReference = ref;
	}
	
	/**
	 * Get the initiator reference
	 * @return the initiator reference
	 */
	public Reference getInitiatorReference() {
		return this.initiatorReference;
	}
	
	/**
	 * Get the final position where the boid needs to be redirected
	 * @return the position of the goal
	 */
	public Point getFinalGoal() {
		return this.finalGoal;
	}
	
	/**
	 * Set the position of the boid's goal
	 * @param p  the position of the goal
	 */
	public void setFinalGoal(Point p) {
		this.finalGoal = p;
	}
	
	/**
	 * Update the weight of an edge of the PRM
	 * @param w  the new edge's weight
	 */
	public void setCurrentVertexWeight(int w) {
		this.currentVertexWeight = w;
	}
	
	/** 
	 * Get the weight of an edge of the PRM
	 * @return  the edge's weight
	 */
	public int getCurrentVertexWeight() {
		return this.currentVertexWeight;
	}
	
	/**
	 * Set the shortest path received as information
	 * @param map  the map with key equal to the starting point and value the next point
	 */
	public void setShortestPath(HashMap<Point, Point> map) {
		this.shortestPathPRM = map;
	}
	
	/**
	 * Get the shortest path of the PRM
	 */
	public HashMap<Point, Point> getShortestPath() {
		return this.shortestPathPRM;
	}
	
	/**
	 * Get all the edges that compose the PRM
	 * @return the map with key equal to a vertex and values equal to all the point to which the key is connected
	 */
	public HashMap<Point, ArrayList<Point>> getEdgesPRM(){
		return this.edgesPRM;
	}
	
	/**
	 * Set the edges of the PRM
	 * @param edges  
	 */
	public void setEdgesPRM(HashMap<Point, ArrayList<Point>> edges) {
		this.edgesPRM = edges;
	}
	
	/**
	 * Update the current boid's subgoal
	 * @param p  the position of the subgoal
	 */
	public void setCurrentSubgoal(Point p) {
		this.currentSubgoal = p;
	}
	
	/**
	 * Get the current boid's subgoal
	 * @return the position of the current subgoal
	 */
	public Point getCurrentSubgoal() {
		return this.currentSubgoal;
	}
	
	/**
	 * Get all the nodes visited by the boid
	 * @return the list of the nodes
	 */
	public ArrayList<Point> getNodeVisited(){
		return this.nodeVisited;
	}
	
	/**
	 * Add a node to the list of the ones already visited by the boid
	 * @param p  the position of the node
	 */
	public void addNodeVisited(Point p) {
		this.nodeVisited.add(p);
	}
	
	/**
	 * Get the weights of the edges of the PRM
	 * @return the map with value = list as [starting_point, destination_point] in increasing order by the weight
	 */
	public HashMap<ArrayList<Point>, Integer> getEdgesWeightPRM(){
		return this.edgesWeightsPRM;
	}
	
	/**
	 * Increase by one the weight of a ginen edge
	 * @param edge  the edge as a list [starting_point, destinaton_point] in increasing order by the weight
	 */
	public void increaseEdgeWeightPRM(ArrayList<Point> edge) {
		this.edgesWeightsPRM.put(edge, this.getEdgesWeightPRM().get(edge)+1);
	}
	
	/**
	 * Get the list of the vertices already visited by the boid
	 * @return the list with vertices
	 */
	public ArrayList<Point> getVertexChecked(){
		return this.vertexAlreadyChecked;
	}
	
	/**
	 * Add a vertex to the list of the ones already checked
	 * @param v  the point to add
	 */
	public void addVertexChecked(Point v) {
		this.vertexAlreadyChecked.add(v);
	}
	
	/**
	 * Get the sheperd's reference 
	 * @return the sheperd's reference
	 */
	public Reference getSheperdReference() {
		return this.sheperdReference;
	}
	
	/**
	 * Set the sheperd's reference
	 * @param ref the sheperd's reference
	 */
	public void setSheperdReference(Reference ref) {
		this.sheperdReference = ref;
	}
	
	/**
	 * Check if the sheperd has already been started 
	 * @return the flag used to check if the sheperd has started
	 */
	public boolean getSheperdStart() {
		return this.sheperdStart;
	}
	
	/**
	 * Set the sheperd as started
	 * @param b  the boolean used as flag
	 */
	public void setSheperdStart(Boolean b) {
		this.sheperdStart = b;
	}
	
	/**
	 * This function will limit the magnitude of the boid's velocity such that it will not go too fast.
	 * @param b 		the boid to which limit the velocity
	 * @param limiter	the value used as a threshold to limit the velocity
	 */
	public void limitVelocity(Boid b, int limiter) {
		if(magnitude(b.getVelocity()) > limiter) {
			b.setVelocity(b.getVelocity().getX()/magnitude(b.velocity), b.getVelocity().getY()/magnitude(b.velocity));
			b.setVelocity(b.getVelocity().getX()*limiter, b.getVelocity().getY()*limiter);
		}
	}
	
	/**
	 * 
	 * @param p		the point used to calculate the magnitude
	 * @return 		the magnitude
	 */
	public double magnitude(Point2D.Double p) {
		return Math.sqrt((Math.pow(p.getX(),2)) + Math.pow(p.getY(), 2));
	}
	
	/**
	 * This function is used to calculate the Euclidian distance between 2 points in 2D
	 * @param p1	first point
	 * @param p2	second point
	 * @return		the distance between the points
	 */
	public double distanceBetweenPoints(Point p1, Point p2) {
		return Math.sqrt(Math.pow((p2.getX() - p1.getX()), 2) + Math.pow((p2.getY() - p1.getY()), 2));
	};

}

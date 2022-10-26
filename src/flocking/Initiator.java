package flocking;

import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.Thread.State;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;
import java.util.ResourceBundle;
import java.util.Set;

import flocking.Initiator;
import interaction.Continue;
import interaction.CoveringStop;
import interaction.Info;
import interaction.Sheperd;
import interaction.SheperdInfo;
import interaction.Update;
import it.unipr.sowide.actodes.actor.Behavior;
import it.unipr.sowide.actodes.actor.CaseFactory;
import it.unipr.sowide.actodes.actor.MessageHandler;
import it.unipr.sowide.actodes.actor.MessagePattern;
import it.unipr.sowide.actodes.actor.Shutdown;
import it.unipr.sowide.actodes.configuration.Configuration;
import it.unipr.sowide.actodes.controller.SpaceInfo;
import it.unipr.sowide.actodes.executor.passive.OldScheduler;
import it.unipr.sowide.actodes.filtering.constraint.IsInstance;
import it.unipr.sowide.actodes.interaction.Kill;
import it.unipr.sowide.actodes.interaction.Stop;
import it.unipr.sowide.actodes.registry.Reference;
import it.unipr.sowide.actodes.service.group.Grouper;
import it.unipr.sowide.actodes.service.logging.ConsoleWriter;
import it.unipr.sowide.actodes.service.logging.Logger;
import it.unipr.sowide.actodes.service.logging.TextualWriter;
import it.unipr.sowide.actodes.service.logging.util.NoCycleProcessing;
import javafx.scene.Node;
import javafx.scene.shape.Circle;

/**
 * @author Mattia Crispino
 */

public class Initiator extends Behavior {
	
	private static final long serialVersionUID = 1L;
	private static final String CONFIG = "resources/configuration";
	private static final int MAP_WIDTH;
	private static final int MAP_HEIGHT;
	private static final int V;										// #vertices of the PRM to create
	private static final int GOAL_DISTANCE;							// distance used as comparison to check if a boid is in the neighbor of the GOAL
	private static final int VERTEX_EDGES;							// total number of outgoing edges that a vertex need to have
	private static final Boolean COVERING;							// flag to enable the covering behaviour
	private static final Boolean GOAL_SEARCH;						// flag to enable the goal searching behaviour
	private static final Boolean SHEPERDING;						// flag to enable the sheperding behaviour
	private static final int SHEPERDING_DISTANCE = 70;				// distance of the boid from the starting position for SHEPERDING
	private static final int VERTEX_MIN_DISTANCE = 50;				// min distance between vertices of the PRM
	private static final int VERTEX_MIN_DISTANCE_OBS = 70;			// min distance between vertices and obstacles of the PRM
	private static final double MAX_VELOCITY = 10.0; 				// upper bound for the generation of the random boid's velocity
	private static final double MIN_VELOCITY = 0.1; 				// lower bound for the generation of the random boid's velocity
	private static final String LOG_FILE_PATH = "log/position/log.txt";
	private static final String LOG_FILE_PRM_VERTICES_PATH = "log/position/prmVertices.txt";
	private static final String LOG_FILE_PRM_EDGES_PATH = "log/position/prmEdges.txt";
	private static final String LOG_DIR_PATH = "log/position";
	private static enum shapes {CIRCLE, RECTANGLE};
	private static final int N_BOID;								// total number of boid to create
	private int stopMsgReceived;									// used to check when to write the position in the log file (it need to be = N_BOID
	private int updateMsgReceived;									// used to check when a new round will start so that the initiator send a CONTINUE msg to the boid and they compute their new position
	private int totalBoidAtGoal;									// used to check how many boid are in the neighbor of the GOAL
	private int totalBoidAtStartSheperding;							// used to check how many boid are at the starting position for SHEPERDING
	private int actualRound;										// log for the actual round of execution
	private Point finalGoal;										// final GOAL that the boids need to reach
	private Reference sheperdReference;								// sheperd's reference
	private Boolean sheperdMoving;									// flag to check if the Sheperd.START msg has been sent (so the sheperd is moving)
	private HashMap<Reference, Boid> boidMap;						// map used to keep track of boids' info
	//private HashMap<Reference, List<Double>> boidInfo;			// map used to keep track of boids' velocity & direction
	private Set<ArrayList<Integer>> positionExtracted;				// set used to keep track of every random position generated (in order to not generate a given position twice)
	private ArrayList<Node> obstacles;
	private ArrayList<ArrayList<Update>> positionLog;				// array used to keep track of the boid position at any given round (at the end this array is used to write the log to an external file)
	private ArrayList<Point> verticesPRM;							// array to store all the vertices of the PRM
	private HashMap<Point, Integer> verticesWeight;					// store the weight of the vertices of the PRM (key=vertex, value=weight)
	private HashMap<Point, ArrayList<Point>> edgesPRM;				// map with key = point and values = to all the other points that this point is connected with an edge 
	private HashMap<Point, Point> mapShortestPath;					// map with key = starting point of the path and value the next point which has the shortest distance from the starting one
	
	private int counter;
	
	static
	  {
	    ResourceBundle b = ResourceBundle.getBundle(CONFIG);
	    MAP_WIDTH = Integer.parseInt(b.getString("map.width"));
	    MAP_HEIGHT = Integer.parseInt(b.getString("map.height"));
	    N_BOID = Integer.parseInt(b.getString("boid"));
	    V = Integer.parseInt(b.getString("vertices"));
	    GOAL_DISTANCE = Integer.parseInt(b.getString("goal.distance"));
	    VERTEX_EDGES = Integer.parseInt(b.getString("vertex.edges"));
	    COVERING = Boolean.parseBoolean(b.getString("covering"));
	    GOAL_SEARCH = Boolean.parseBoolean(b.getString("goal.searching"));
	    SHEPERDING = Boolean.parseBoolean(b.getString("sheperding"));
	  }
	
	private static final MessagePattern STOP = MessagePattern.contentPattern(new IsInstance(Stop.class));
	private static final MessagePattern UPDATE = MessagePattern.contentPattern(new IsInstance(Update.class));
	private static final MessagePattern COVERING_STOP 	= MessagePattern.contentPattern(new IsInstance(CoveringStop.class));
	
	/**
	 * Class constructor
	 * @param obstacles		list of the obstacles in the Map
	 */
	public Initiator(ArrayList<Node> obstacles) {
		this.stopMsgReceived = 0;
		this.updateMsgReceived = 0;
		this.actualRound = 0;
		this.totalBoidAtGoal = 0;
		this.totalBoidAtStartSheperding = 0;
		this.sheperdReference = null;
		this.sheperdMoving = false;
		this.finalGoal = new Point();
		this.boidMap = new HashMap<>();
		this.obstacles = obstacles;
		this.positionExtracted = new HashSet<>();
		this.positionLog = new ArrayList<ArrayList<Update>>();
		this.verticesPRM = new ArrayList<>();
		this.verticesWeight = new HashMap<>();
		this.edgesPRM = new HashMap<>();
		this.counter = 0;
	}
	

	@Override
	public void cases(CaseFactory c) {
		
		// COVERING HANDLER
		MessageHandler cov_stop = (m) -> { 
			/* Kill the boid and then send a stop in loopback such that the initiator will log the file
			 */
			send(SPACE, Kill.KILL);
			send(this.getReference(), Stop.STOP);
			return null;
		};
		c.define(COVERING_STOP, cov_stop);

		// STOP HANDLER => write to log file the position of the boid for any given round
		MessageHandler stop = (m) -> {
			//this.stopMsgReceived +=1;
			//if(this.stopMsgReceived == N_BOID) {
				System.out.println("STOP RECEIVED. WRITE TO FILE");
				// check for dir and file to log if exists
				File dir = new File(LOG_DIR_PATH);
				if(!dir.exists()) {
					System.out.println("log directory created");
					dir.mkdir();
				}
				try {
					File file = new File(LOG_FILE_PATH);
					if(!file.exists()) {
						System.out.println("log file created");
						file.createNewFile();
					}
					FileWriter fw = new FileWriter(file);
					BufferedWriter bw = new BufferedWriter(fw);
					// write the logArray to the file
					for (ArrayList<Update> arr : this.positionLog) {
						for (Update update : arr) {
							String line = "";
							line += update.getSenderReference().toString() + "," + (int) update.getPosition().getX() + ","+(int) update.getPosition().getY() + "," + (int) update.getVelocity().getX() +","+(int) update.getVelocity().getY();
							bw.write(line);
							bw.newLine();
						}
						// write a new line to separate one round to the successive
						bw.write("-");
						bw.newLine();
					}
					bw.close();
				} catch (IOException e) {
					e.printStackTrace();
				}	
				
				System.out.println("Terminating the thread INITIATOR");
				if(Thread.currentThread().getState() == State.RUNNABLE) {
					try {
						Thread.sleep(600000);
					} catch (InterruptedException e) {
					}
				}
			//}
			
			return Shutdown.SHUTDOWN; // kill the initiator
		};
		c.define(STOP, stop);
		
		/* UPDATE HANDLER 
		 * Once the initiator receives an "update" msg, it will wait until the number of updates messages received are equal to the number of the boids;
		 * then, based on what behaviour flag is enable (sheperding, covering ...) it will act differently
		 */ 
		MessageHandler update = (m) -> {
			Update msg = (Update) m.getContent();
			this.updateMsgReceived += 1;
			// get the array at index "round", add this update msg and push back the array at index "round"
			if(this.positionLog.size() < (msg.getRound() + 1)) {
				this.positionLog.add(msg.getRound(), new ArrayList<>(Arrays.asList(msg)));
			} else {
				ArrayList<Update> tmp = this.positionLog.get(msg.getRound());
				tmp.add(msg);
				this.positionLog.remove(msg.getRound());
				this.positionLog.add(msg.getRound(), tmp);
			}			
			
			int totBoid = !SHEPERDING ? N_BOID : (N_BOID+1);
			// the initiator has received all update messages of this round 
			if(this.updateMsgReceived == totBoid) {
				this.actualRound = msg.getRound();
				if(SHEPERDING) { // SHEPERDING behaviour
					// check how many boids are at the starting position (vertex 1)
					ArrayList<Update> arr = this.positionLog.get(this.actualRound);
					for (Update pos : arr) {
						// if the sheperd is not moving => check how many boid are at the starting position
						if(!this.sheperdMoving) {
							// check the distance of the boid from the starting position (vertex with lowest weight) and check that the boid is not the sheperd
							if(pos.getPosition().distance(this.verticesPRM.get(this.verticesPRM.size()-1)) < SHEPERDING_DISTANCE && (pos.getSenderReference() != this.getSheperdReference())) {
								this.totalBoidAtStartSheperding += 1;
							}
						} else if(this.sheperdMoving) {
							// the sheperd is moving => check how many boid are at the final goal
							if(pos.getPosition().distance(this.finalGoal) < GOAL_DISTANCE && (pos.getSenderReference() != this.getSheperdReference())) {
								this.totalBoidAtGoal += 1;
							}
						}
						
					}
					int totBoidtoCheck = !this.sheperdMoving ? this.totalBoidAtStartSheperding : this.totalBoidAtGoal;
					// check if all the boid are either at the starting point or at the final goal and the sheperd is not moving
					if(this.totalBoidAtStartSheperding == N_BOID) { 
						if(!this.sheperdMoving) {
							// tell the sheperd to start moving
							send(SPACE, Sheperd.START);
							this.updateMsgReceived = 0;
							this.totalBoidAtStartSheperding = 0;
							send(SPACE, Continue.CONTINUE);
							this.sheperdMoving = true;
						} else {
							this.updateMsgReceived = 0;
							this.totalBoidAtStartSheperding = 0;
							send(SPACE, Continue.CONTINUE);
							this.counter +=1;
						}
					} else if(this.totalBoidAtGoal >= ((90*N_BOID)/100)) { // stop the execution if at least 90% of the boid are at the goal
						send(SPACE, Kill.KILL);
						send(this.getReference(), Stop.STOP);
					} else {	// continue the execution
						this.updateMsgReceived = 0;
						this.totalBoidAtStartSheperding = 0;
						send(SPACE, Continue.CONTINUE);
					}
				}
				else if(!COVERING) {
					/* firstly check if all the boid are at the goal (ONLY IF IT'S	NOT SELECTED THE COVERING BEHAVIOUR)
					 * This is done by computing the distance from the goal's center and every boids' position and if all the boids (or a subset of them such 70%) have a distance
					 * < GOAL_DISTANCE so send a KILL message to the boid to stop their execution
					 */
					ArrayList<Update> arr = this.positionLog.get(this.actualRound);
					for (Update pos : arr) {
						// check the distance of the boid from the GOAL
						if(pos.getPosition().distance(this.finalGoal) < GOAL_DISTANCE) {
							this.totalBoidAtGoal += 1;
						}
					}
					
					if(this.totalBoidAtGoal >= ((80*N_BOID)/100)) { // if a given % of the boid are at the GOAL => send the KILL message such that the boid stops its execution
						send(SPACE, Kill.KILL);
						send(this.getReference(), Stop.STOP);
					} else {
						this.updateMsgReceived = 0;
						this.totalBoidAtGoal = 0;
						send(SPACE, Continue.CONTINUE);
					}
				} else {
					this.updateMsgReceived = 0;
					// if the COVERING behaviour is selected, then send a CONTINUE message
					send(SPACE, Continue.CONTINUE);
				}
			}
			return null;
		};
		c.define(UPDATE, update);

		// START code for the Initiator
		MessageHandler h = (m) -> {
			System.out.println("Initiator START");
			generatePRM();
			findShortestPath();
			
			// create N_BOID actor "BOID" and set them a random position in [(0,0), (MAP_WIDTH, MAP_HEIGHT)]
			Random random;
			for (int i = 0; i < N_BOID; i++) {
				random = new Random(i);
				int x = 0;
				int y = 0; 
				Boolean invalidPosition = false;
				
				// (1) check if the position has not been already generated && (2) if the position is not inside an obstacle
				do {
					x = random.nextInt(0,MAP_WIDTH);
					y = random.nextInt(0,MAP_HEIGHT);
					if(positionExtracted.contains(Arrays.asList(x,y))) {
						invalidPosition = true;
						System.out.println("position " + x +","+y + " has already been generated");
					} else {
						// check for obstacles
						invalidPosition = checkPositionInsideObstacle(Initiator.shapes.CIRCLE, x, y);
					}
				} while (invalidPosition);
				
				double direction = random.nextDouble(0, 361);		// random direction in [0,360] degrees
				Point2D.Double velocity = new Point2D.Double(random.nextDouble(MIN_VELOCITY, MAX_VELOCITY), random.nextDouble(MIN_VELOCITY, MAX_VELOCITY));
				// create a new actor in a random position in the map
				Point p = new Point(x, y);
				Reference ref = actor(new Boid(p, velocity, direction));	// create actor in position p with random velocity and random direction 
				// send the position to the boid created (they are not the sheperd so set to false)
				send(ref, new Info(p,velocity,direction,ref, obstacles, this.verticesPRM, this.mapShortestPath, this.edgesPRM, false));
				// save the info about the actor in order to later send these in broadcast to all other actors
				boidMap.put(ref, new Boid(p, velocity, direction));
				
				// UPDATE the positionLog for the 1st round (index 0)
				if(this.positionLog.size() == 0) {
					this.positionLog.add(0, new ArrayList<>(Arrays.asList(new Update(ref, p, velocity, direction, 0, null))));
				} else {
					ArrayList<Update> tmp = this.positionLog.get(0);
					tmp.add(new Update(ref, p, velocity, direction, 0, null));
					this.positionLog.remove(0);
					this.positionLog.add(0, tmp);
				}
			}
			
			// send a message in BROADCAST to inform all the other boids about the position of the new one (such that every boid can update its own list in which it
			// keeps track of all the other boids and then finds the ones that are in its neighborhood) 
			for (var entry : boidMap.entrySet()) {
				// send the position (x,y) and the actor's reference in broadcast
				send(SPACE, new Info(entry.getValue().getBoidPosition(), entry.getValue().getVelocity(), entry.getValue().getDirection(), entry.getKey(), obstacles, 
						this.verticesPRM, this.mapShortestPath, this.edgesPRM, false));
			}
			
			// create the SHEPERD "boid" in position (0,0), send its info to other boids
			if(SHEPERDING) {
				Random r = new Random();
				double direction = r.nextDouble(0, 361);		// random direction in [0,360] degrees
				Point2D.Double velocity = new Point2D.Double(r.nextDouble(MIN_VELOCITY, MAX_VELOCITY), r.nextDouble(MIN_VELOCITY, MAX_VELOCITY));
				// create the sheperd that is aligned with the starting position (same y but with an offset in x)
				Point startingPoint = this.verticesPRM.get(this.verticesPRM.size()-1);
				
				/* Define the correct position for the sheperd*/
				Point startPoint = this.verticesPRM.get(this.verticesPRM.size()-1);
				Point nextPoint  = this.mapShortestPath.get(startPoint);
				Point p = new Point();
				if(nextPoint.getY() < startingPoint.getY()) { // offset to set the correct poisition of the sheperd with respect to the starting point
					p.x = (int) (startingPoint.getX()-100);
					p.y = (int) (startingPoint.getY()+100);
				} else if(nextPoint.getY() > startingPoint.getY()) {
					p.x = (int) (startingPoint.getX()-100);
					p.y = (int) (startingPoint.getY()-100);
				}
				Reference ref = actor(new Boid(p, velocity, direction));	// create actor in position p with random velocity and random direction
				
				// UPDATE the positionLog for the 1st round (index 0)
				if(this.positionLog.size() == 0) {
					this.positionLog.add(0, new ArrayList<>(Arrays.asList(new Update(ref, p, velocity, direction, 0, null))));
				} else {
					ArrayList<Update> tmp = this.positionLog.get(0);
					tmp.add(new Update(ref, p, velocity, direction, 0, null));
					this.positionLog.remove(0);
					this.positionLog.add(0, tmp);
				}
				// save the reference of the sheperd
				this.setSheperdReference(ref);
				
				// send all the information to the SHEPERD created
				send(ref, new Info(p,velocity,direction,ref, obstacles, this.verticesPRM, this.mapShortestPath, this.edgesPRM, true));
				
				// save the info about the SHEPERD in order to later send these in broadcast to all other actors
				boidMap.put(ref, new Boid(p, velocity, direction));
				// send a SheperdInfo msg to the boids to inform them about the properties of the sheperd
				send(SPACE, new SheperdInfo(ref, p, velocity));
			} 
			
			// after creating N actor, the initiator send a message in broadcast to all the other boids such that this message means "I (initiator) have created N boids and so you (boid)
			// can start your own behaviour but first find M boid that compose your neighborhood
			send(SPACE, Continue.CONTINUE); 
			
			
			
			
			return null;
		};
		c.define(START, h);
		
		c.define(ACCEPTALL, DUMMY);
		
	}
	
	/*
	 * This method will generate the probabilist roadmap, creating a given number of edges and then finding the edges
	 */
	private void generatePRM() {
		int totVertex = 0;
		Random r = new Random();
		Boolean invalidVertex = null;
		while (this.verticesPRM.size() < V) {
			// create a random vertex
			invalidVertex = false;
			int x = r.nextInt(50,MAP_WIDTH-50);  // offset 50 such that the vertices are not at the boundaries of the map
			int y = r.nextInt(50,MAP_HEIGHT-50);
			Point v = new Point(x, y);
			
			if(this.verticesPRM.size() == 0) {
				//System.out.println("[LOG PRM] Empty array. Added " + v.toString());
				for (Node node : obstacles) {	// (3) check min distance from obstacles
					if(node instanceof Circle) {
						Circle obs = (Circle) node;
						Point c = new Point((int) obs.getCenterX(), (int) obs.getCenterY());
						if((distanceBetweenPoints(v, c) - obs.getRadius()) < VERTEX_MIN_DISTANCE_OBS) {
							System.out.println("[LOG PRM] Distance between obstacle " + obs.toString() + " and new vertex " + v.toString() + " is < MIN_DISTANCE (" + (distanceBetweenPoints(v, c) - obs.getRadius()) + ")" );
							invalidVertex = true;
							break;
						} 
					}
				}

			} else {
				// loop to every vertex store to check the constraint of (1) no duplicate vertex (2) min distance from other vertices (3) min distance from obstacles
				for (Point p : this.verticesPRM) {
					if(p.equals(v)) {	// (1) check for duplicates
						invalidVertex = true;
						//System.out.println("[LOG PRM] Point " + p.toString() + " is equal to new vertex " + v.toString());
						break;
					} else if(distanceBetweenPoints(v, p) < VERTEX_MIN_DISTANCE) {	 // (2) check min distance
						invalidVertex = true;
						break;
					} else {
						for (Node node : obstacles) {	// (3) check min distance from obstacles
							if(node instanceof Circle) {
								Circle obs = (Circle) node;
								Point c = new Point((int) obs.getCenterX(), (int) obs.getCenterY());
								if((distanceBetweenPoints(v, c) - obs.getRadius()) < VERTEX_MIN_DISTANCE_OBS) {
									invalidVertex = true;
									break;
								} else if((Math.pow((p.getX()-c.getX()), 2) + Math.pow((p.getY()+c.getY()), 2)) <= (Math.pow(obs.getRadius(), 2))) { // check if the point is inside an obstacle so (x-xc)^2+(y-yc)^2 <= r^2 with point=(x,y) and circle with center (xc,yc)
									System.out.println("[LOG PRM] The point is inside the obstacle " + obs.toString());
									invalidVertex = true;
									break;
								}
							}
						}
					}
				}
			}	
			if(!invalidVertex) { // the generated vertex is valid
				this.verticesPRM.add(v);
				// initialize the map
				edgesPRM.put(v, new ArrayList<>());
			}
		}
		
		// SORTING of the vertices in order to give to each one an increasing weigth
		sortVerticesPRM();

		// find the edges
		for (Point p0 : this.verticesPRM) {
			double x1 = p0.getX();
			double y1 = p0.getY();
			HashMap<Point, Integer> mapDistance = new HashMap<>();	// map which store as values the distance from this point (p0) and the key of the map
			for (Point p1 : this.verticesPRM) {
				boolean intersectObstacle = false;
				double x2 = p1.getX();
				double y2 = p1.getY();
				if(!p0.equals(p1)) {
					for (Node node : obstacles) {
						if(node instanceof Circle) {
							Circle obstacle = (Circle) node;
							Point2D.Double center = new Point2D.Double(obstacle.getCenterX(), obstacle.getCenterY());
							Line2D line = new Line2D.Double(p0, p1);
							if(line.ptSegDist(center) < obstacle.getRadius()) {
								//System.out.println("Line " + line.toString() + " intersect " + obstacle.toString());
								intersectObstacle = true;
							}
						}
					}
					if(!intersectObstacle) {
						// For every point, find the edges for the 4 nearest point of this one.
						mapDistance.put(p1, (int) p0.distance(p1)); // store in the map the distance to this point
					}
				}
			}
			
			/* save, for this starting point, the VERTEX_EDGES nearest point to use to find the edges
			 * NB: every vertex has AT LEAST an edge connecting to a vertex with higher weight because, since the weight increases as a vertex becomes closer to the goal,
			 * in this way I'll make sure that there is at least a path which goes from the vertex with the lowest weight (the fartest from the goal) to the vertex with
			 * highest weight (the goal)
			 */
			ArrayList<Point> nearestPoint = new ArrayList<>();
			int pointFound = 0;		
			boolean weightPoint = false;	// used as flag to be sure that at least one node with weight > this node is included in the nodes for the edges
			while(pointFound < VERTEX_EDGES) {
				// if this node is the GOAL => select the VERTEX_EDGES nodes without considering to have at least one node with greater weight because it doesn't exists
				if(this.verticesWeight.get(p0) == this.verticesWeight.size())
				{
					double minDistance = Integer.MAX_VALUE;
					Point closer = new Point();
					for (var entry : mapDistance.entrySet()) {
						if(entry.getValue() < minDistance) {
							minDistance = entry.getValue();
							closer = entry.getKey();
						}
					}
					nearestPoint.add(closer);
					mapDistance.remove(closer);
					pointFound+=1;
				} else {
						double minDistance = Integer.MAX_VALUE;
						Point closer = new Point();
						for (var entry : mapDistance.entrySet()) {
							//System.out.println("[INITIATOR] Actual point used to check the distance: " + this.verticesWeight.get(entry.getKey()));
							if(entry.getValue() < minDistance) {
								minDistance = entry.getValue();
								closer = entry.getKey();
							}
						}
						// check if the point which has minDistance has the weight > this point
						if(this.verticesWeight.get(closer) > this.verticesWeight.get(p0)) {
							weightPoint = true;
						}
						//System.out.println("Current weight: " + this.verticesWeight.get(p0) + " nearest weight: " + this.verticesWeight.get(closer));
						if(weightPoint) {
							nearestPoint.add(closer);
							mapDistance.remove(closer);
							pointFound+=1;
						} else if(pointFound >= (VERTEX_EDGES-1)){ // leave at least an outgoind edge to a point which has a greater weight than this one
							mapDistance.remove(closer);
						} else {
							// add the point
							nearestPoint.add(closer);
							mapDistance.remove(closer);
							pointFound+=1;
						}
						
					}
			} 
			// save the destination point for the outgoing edges of this point
			edgesPRM.put(p0, nearestPoint);
		}
		
		for(var entry : edgesPRM.entrySet()) {
			Point start = entry.getKey();
			for (Point p : entry.getValue()) {
				System.out.println("Point " + this.verticesWeight.get(start) + " is connected to " + this.verticesWeight.get(p));
			}
			System.out.println("\n");
			
		}
		
		
		// WRITE VERTICES to external file
		File dir = new File(LOG_DIR_PATH);
		if(!dir.exists()) {
			System.out.println("log directory created");
			dir.mkdir();
		}
		try {
			File file = new File(LOG_FILE_PRM_VERTICES_PATH);
			if(!file.exists()) {
				System.out.println("log file created");
				file.createNewFile();
			}
			FileWriter fw = new FileWriter(file);
			BufferedWriter bw = new BufferedWriter(fw);
			// loop through vertices
			for (Point p : this.verticesPRM) {
				String line = "";
				line += (int)p.getX() +","+(int)p.getY();
				bw.write(line);
				bw.newLine();
			}
			bw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		// WRITE EDGES
		try {
			File file = new File(LOG_FILE_PRM_EDGES_PATH);
			if(!file.exists()) {
				System.out.println("log file created");
				file.createNewFile();
			}
			FileWriter fw = new FileWriter(file);
			BufferedWriter bw = new BufferedWriter(fw);
			for(var entry : edgesPRM.entrySet()) {
				ArrayList<Point> tmp = entry.getValue();
				for (Point point : tmp) {
					String line = (int)entry.getKey().getX() + "," + (int)entry.getKey().getY() + "," + (int)point.getX() + "," + (int)point.getY(); 
					bw.write(line);
					bw.newLine();
				}
			}
			bw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
	
	/*
	 * Sort the vertices based on their distance from the bottom-rigth edge of the map
	 * Greater the distance, the lower the weight.
	 * The MAX weight will be equal to number of vertices and the MIN weight will be 1.
	 */
	private void sortVerticesPRM() {
		ArrayList<Point> pointTmp = (ArrayList<Point>) this.verticesPRM.clone();
		this.verticesPRM.clear();

		Point STOP = new Point(MAP_WIDTH, MAP_HEIGHT);
		while(pointTmp.size() > 0) {
			double minDistance = Integer.MAX_VALUE;
			int minIndex = 0;
			for (Point p : pointTmp) {
				double distance = STOP.distance(p);
				if(distance < minDistance) {
					minIndex = pointTmp.indexOf(p);
					minDistance = distance;
				}
			}
			this.verticesPRM.add(pointTmp.get(minIndex));
			pointTmp.remove(minIndex);
		}
		System.out.println("\n\nSorting complete. Vertices: ");
		int max_weigth = V;
		for (Point point : this.verticesPRM) {
			System.out.println(point.toString() + " weight " + max_weigth);
			// update the map
			this.verticesWeight.put(point, max_weigth);
			max_weigth -=1;
		}
		this.finalGoal = this.verticesPRM.get(0);
		System.out.println("FINAL GOAL: " + this.finalGoal.toString());
		System.out.println("\n\n");
	}
	
	/*
	 * This method will find the shortest path from a starting node to the destination (goal) node.
	 * For every vertex 'i' in the PRM, it will compute the distance from 'i' to 'i+j' with j=i+1,...,V (V is the number of vertices of the PRM) and store the minimum distance and the index of the 
	 * corresponding vertex; in this way, the minimum distance is computed for only those vertices that have a weight greater than vertex 'i'.
	 * After that, it will check if the vertex found is a vertex connected with an outgoind edge from the vertex 'i' (this is possibile since there's the map "edgesPRM" which has, as key, the vertex and 
	 * as values the VERTEX_EDGES vertices to which it's connected); if it's not, it will check the next closer vertex and then check again and so on.
	 */
	private void findShortestPath() {
		this.mapShortestPath = new HashMap<>();
		for (var entry : this.edgesPRM.entrySet()) {
			Point start = entry.getKey();
			// don't find the shortest path if the actual point is the goal (the point with max weight)
			if(this.verticesWeight.get(start) != this.verticesWeight.size())
			{
				// loop through every node connected to this "start" node
				Point closer = new Point();
				double minDistance = Integer.MAX_VALUE;
				for (Point p : entry.getValue()) {
					// compute the distance only if p's weight > "start" node
					if(this.verticesWeight.get(p) > this.verticesWeight.get(start)) {
						double distance = p.distance(start);
						if(distance < minDistance) {
							minDistance = distance;
							closer = p;
						}
					}
				}
				/* MAP UPDATE
				 * key = starting point
				 * value = the point found which has the shortest distance from the starting one
				 */
				this.mapShortestPath.put(start, closer);
			}
		}
		System.out.println("\n");
		for (var entry : this.mapShortestPath.entrySet()) {
			System.out.println("Point " + this.verticesWeight.get(entry.getKey()) + " has shortest path with " + this.verticesWeight.get(entry.getValue()) );
		}
	}

	
	/*
	 * Check if the point, which coordinates are the parameters x y, is inside an obstacle
	 */
	private Boolean checkPositionInsideObstacle(shapes shape, int x, int y) {
		switch (shape) {
		case CIRCLE:
			// loop every circle in the "obstacles" list and check if the position is inside this
			for (Node node : this.getObstacles()) {
				if(node instanceof Circle) {
					Circle c = (Circle) node;
					// eq. circle:  (x-x_center)^2+(y-y_center)^2 = r^2
					if((Math.pow((x-c.getCenterX()), 2) + Math.pow(y-c.getCenterY(), 2)) <= (Math.pow(c.getRadius(), 2))) {
						System.out.println("position " + x +","+y + " is inside the circle " + c.getCenterX() + ","+c.getCenterY() + " with radius " + c.getRadius());
						return true;
					}
				}
			}
			
			break;
		}
		return false;
	}
	
	/**
	 * Get the list of the obstacles in the map
	 * @return the list of all the obstacles in the map
	 */
	public ArrayList<Node> getObstacles() {
		return this.obstacles;
	}
	
	/**
	 * Get the sheperd's reference (useful if the "sheperd" property, in the configuration file, is set to true)
	 * @return the sheperd's reference
	 */
	public Reference getSheperdReference() {
		return this.sheperdReference;
	}
	
	/**
	 * Set the sheperd's reference
	 * @param ref  the sheperd's ref
	 */
	public void setSheperdReference(Reference ref) {
		this.sheperdReference = ref;
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
	

	/**
	 * Starts the standalone version of the application
	 **/
	public static void main(final ArrayList<Node> obstacles) {

		Configuration c = SpaceInfo.INFO.getConfiguration();

		c.setFilter(Logger.OUTPUTMESSAGE);
		c.setLogFilter(new NoCycleProcessing());

		c.addWriter(new TextualWriter("core/flocking"));
		c.addWriter(new ConsoleWriter());

		c.addService(new Grouper());
		c.setExecutor(new OldScheduler(new Initiator(obstacles)));
		
		c.start();
	}

	

}

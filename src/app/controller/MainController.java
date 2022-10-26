package app.controller;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.ResourceBundle;
import java.util.Set;

import app.MainFX;
import flocking.Initiator;
import javafx.animation.Animation;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.ParallelTransition;
import javafx.animation.SequentialTransition;
import javafx.animation.Timeline;
import javafx.animation.TranslateTransition;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.ButtonBar;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.text.Text;
import javafx.util.Duration;

/**
 * @author Mattia Crispino
 */

public class MainController extends Thread implements Initializable {

	private static final String CONFIG = "resources/configuration";
	private static final int N_BOID;
	private static final Boolean SHEPERDING;						// flag to enable the sheperding behaviour
	private static final int MAP_WIDTH;
	private static final int MAP_HEIGHT;
	private static final int VERTICES;
	private static final int RADIUS_PRM = 10;
	private static final int RADIUS_BOID = 2;
	private static String SheperdReference = "";
	private static final String LOG_FILE_PRM_VERTICES_PATH = "log/position/prmVertices.txt";
	private static final String LOG_FILE_PRM_EDGES_PATH = "log/position/prmEdges.txt";
	
	static
	  {
	    ResourceBundle b = ResourceBundle.getBundle(CONFIG);

	    MAP_WIDTH = Integer.parseInt(b.getString("map.width"));
	    MAP_HEIGHT = Integer.parseInt(b.getString("map.height"));
	    N_BOID = Integer.parseInt(b.getString("boid"));
	    VERTICES = Integer.parseInt(b.getString("vertices"));
	    SHEPERDING = Boolean.parseBoolean(b.getString("sheperding"));
	  }

	private int TIMER = 5000;
	private Parent map1, map2, map3;
	@FXML BorderPane rootLayout;
	@FXML Pane centerPane;
	@FXML ButtonBar buttonBar;
	@FXML Text roundTxt;
	
	private Scene scene;
	ArrayList<Point> position 				      = new ArrayList<>();
	ArrayList<String> circleID 					  = new ArrayList<>();
	private static ArrayList<Node> obstacles 	  = new ArrayList<>();
	private static ArrayList<Circle> verticesPRM  = new ArrayList<>();
	private static ObservableList<Node> nodesMap1 = FXCollections.observableArrayList();
	private static ObservableList<Node> nodesMap2 = FXCollections.observableArrayList();
	private static ObservableList<Node> nodesMap3 = FXCollections.observableArrayList();
	
	private Parent loadScene(String sc) throws IOException {
        return FXMLLoader.load(MainFX.class.getResource(sc));
    }

	public void loadMap1(ActionEvent event) throws IOException {
		obstacles = new ArrayList<>();
		//get all the children in the map
		scene = centerPane.getScene();
		scene.getRoot().applyCss();
		
		// remove any other obstacles from a previous map (if that map were loaded before this one such that there's not overlapping of different obstacles)
		Set<Node> obsToRemove = (Set<Node>) centerPane.lookupAll(".circle");
		System.out.println("Previous obstacles found " + obsToRemove.toString());
		if(!obsToRemove.isEmpty()) {
			for (Node node : obsToRemove) {
				centerPane.getChildren().remove(node);
				obstacles.remove(node);
			}
		}
		
		// add all the children to the existing pane
		for (Node node : nodesMap1) {
			centerPane.getChildren().add(node);
		}
		System.out.println("Map 1 node: " + nodesMap1.toString());
		
		Set<Node> circles = (Set<Node>) centerPane.lookupAll(".circle");
		for (Node node : circles) {
			obstacles.add(node);
		}
		
		/* LOG
		for (Node obs : obstacles) {
			System.out.println(obs.toString());
		} */

	}
    
	public void loadMap2(ActionEvent event) throws IOException {
		obstacles = new ArrayList<>();
		scene = centerPane.getScene();
		scene.getRoot().applyCss();
		// remove any other obstacles from a previous map (if that map were loaded before this one such that there's not overlapping of different obstacles)
		Set<Node> obsToRemove = (Set<Node>) centerPane.lookupAll(".circle");
		System.out.println("Previous obstacles found " + obsToRemove.toString());
		if(!obsToRemove.isEmpty()) {
			for (Node node : obsToRemove) {
				centerPane.getChildren().remove(node);
				obstacles.remove(node);
			}
		}
		// add all the children to the existing pane
		for (Node node : nodesMap2) {
			centerPane.getChildren().add(node);
		}
		System.out.println("Map 2 node: " + nodesMap2.toString());
		
		Set<Node> circles = (Set<Node>) centerPane.lookupAll(".circle");
		for (Node node : circles) {
			obstacles.add(node);
		}
		
		/* LOG
		for (Node obs : obstacles) {
			System.out.println(obs.toString());
		} */
	}
	
	public void loadMap3(ActionEvent event) {
		obstacles = new ArrayList<>();
		scene = centerPane.getScene();
		scene.getRoot().applyCss();
		// remove any other obstacles from a previous map (if that map were loaded before this one such that there's not overlapping of different obstacles)
		Set<Node> obsToRemove = (Set<Node>) centerPane.lookupAll(".circle");
		System.out.println("Previous obstacles found " + obsToRemove.toString());
		if(!obsToRemove.isEmpty()) {
			for (Node node : obsToRemove) {
				centerPane.getChildren().remove(node);
				obstacles.remove(node);
			}
		}
		// add all the children to the existing pane
		for (Node node : nodesMap3) {
			centerPane.getChildren().add(node);
		}
		System.out.println("Map 2 node: " + nodesMap3.toString());
				
		ObservableList<Node> nodes = map3.getChildrenUnmodifiable();
		System.out.println("Map 3 node: " + nodesMap3.toString());

		
		Set<Node> circles = (Set<Node>) centerPane.lookupAll(".circle");
		for (Node node : circles) {
			obstacles.add(node);
		}
		/* LOG
		for (Node obs : obstacles) {
			System.out.println(obs.toString());
		} */

	}
	
	public void startSimulation() {
		System.out.println("simulation started");

		/* pass to the initiator a list which contain every obstacle referenced above
		 * such that it will be able to inform the boid about every obstacles' position
		 */
		System.out.println("obstacles: " + obstacles.size());
		// separate thread for the Initiator
		Thread initStart = new Thread() {
			@Override
			public void run() {
				Initiator.main(obstacles);
			}
		};
		initStart.start();
		
		// wait for the initiator's thread to finish
		while(initStart.getState() != State.TIMED_WAITING) {
			try {
				Thread.sleep(TIMER);
			} catch (InterruptedException e) {
			}
		}
	
		initStart.interrupt();
		System.out.println("[MAIN CONTROLLER] Thread INITIATOR has terminated");
		// call the method to load the log file and then move all the boids
		try {
			drawPRM();
			drawBoid();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public void drawPRM() {
		String file_path = "log/position/prm.txt";
		String dir_path = "log/position";
		File dir = new File(dir_path);
		if(!dir.exists()) {
			System.out.println("directory created");
			dir.mkdir();
		}
		
		File file = new File(LOG_FILE_PRM_VERTICES_PATH);
		if(!file.exists()) {
			System.out.println("file created");
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		if(file.exists()) {
			try {
				int weight = VERTICES;
				BufferedReader br = new BufferedReader(new FileReader(file));
				String line;
				while(true) {
					line = br.readLine();
					if(line != null) {
						int x = Integer.parseInt(line.split(",")[0]);
						int y = Integer.parseInt(line.split(",")[1]);
						Circle v = new Circle(x,y,RADIUS_PRM);
						v.setFill(Color.MAGENTA);
						Text t = new Text(v.getCenterX(), v.getCenterY(), Integer.toString(weight));
						centerPane.getChildren().add(v);
						centerPane.getChildren().add(t);
						verticesPRM.add(v);
						weight-=1;
					} else {
						break;
					}
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		// DRAW EDGES
		file = new File(LOG_FILE_PRM_EDGES_PATH);
		if(!file.exists()) {
			System.out.println("file created");
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		if(file.exists()) {
			try {
				BufferedReader br = new BufferedReader(new FileReader(file));
				String line;
				while(true) {
					line = br.readLine();
					if(line != null) {
						int x0 = Integer.parseInt(line.split(",")[0]);
						int y0 = Integer.parseInt(line.split(",")[1]);
						int x1 = Integer.parseInt(line.split(",")[2]);
						int y1 = Integer.parseInt(line.split(",")[3]);
						Line edge = new Line(x0,y0,x1,y1);
						edge.setFill(Color.BLACK);
						edge.setOpacity(.1);
						centerPane.getChildren().add(edge);
					} else {
						break;
					}
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		
	}
	
	/**
	 * Thanks to this method it's possible to have a visual representation of the simulation computed.
	 * It will read from the log file all the position of the boids and then it will draw, for any given round of computation,
	 * the boids at their position (their movement is achieved thanks to JavaFX transitions).
	 * If the SHEPERDING property is enabled, them it will draw also the sheperd as a red circle with radius greater than the radius 
	 * of the boids.
	 * @throws InterruptedException
	 */
	public void drawBoid() throws InterruptedException {
		String file_path = "log/position/log.txt";
		String dir_path = "log/position";
		File dir = new File(dir_path);
		if(!dir.exists()) {
			System.out.println("directory created");
			dir.mkdir();
		}
		
		File file = new File(file_path);
		if(!file.exists()) {
			System.out.println("file created");
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		int step = (!SHEPERDING) ? (N_BOID+1) : (N_BOID+2);
		try {
			SheperdReference = getSheperdReference(file, step);
			System.out.println("SHEPERD REFERENCE: " + SheperdReference); 
		} catch (IOException e1) {
			e1.printStackTrace();
		}
			
		
		if(file.exists()) {
			try {
				BufferedReader br = new BufferedReader(new FileReader(file));
				String line;
				int counter = 0;
				int totRound = 0;
				ObservableList<Node> obsList = centerPane.getChildren();
				HashMap<String, Node> logNodes = new HashMap<>();
				
				SequentialTransition seqTrans = new SequentialTransition();
				ParallelTransition parTrans = new ParallelTransition();	
				while(true) {
					line = br.readLine();
					counter +=1;
					if(counter % (step) != 0) {		
						if(line != null && !Character.toString(line.charAt(0)).equals("-")) {
							// get the position and reference from the log file
							String reference = line.split(",")[0].split("@")[0].split("\\.")[0];	// get the ID formatted as 00000xxx
							final int round = totRound;
							int goalX = Integer.parseInt(line.split(",")[1]);
							int goalY = Integer.parseInt(line.split(",")[2]);
							int vx = Integer.parseInt(line.split(",")[3]);
							int vy = Integer.parseInt(line.split(",")[4]);
							
							// create the new node ad set its ID
							String id = "#"+reference;
							Circle node = (Circle) centerPane.lookup(id);
							if(node == null) {
								Circle c;
								// check if the node with this reference is the sheperd
								if(reference.equals(SheperdReference)) {
									c = new Circle(goalX,goalY,2*RADIUS_BOID);
									c.setFill(Color.RED);
								} else {
									// create the new node ad set its ID
									c = new Circle(goalX,goalY,RADIUS_BOID);
								}
								c.setId(reference);
								// the node is not a child so add it to the children
								centerPane.getChildren().add(c);
							} else {
								Timeline timeline = new Timeline(new KeyFrame(Duration.millis(200),
							            new KeyValue(node.centerXProperty(), goalX),
							            new KeyValue(node.centerYProperty(), goalY)));
								// update the text to display the actual round of execution
							    timeline.setOnFinished(actionEvent -> {
							    	roundTxt.setText(Integer.toString(round));
							    });
							    parTrans.getChildren().add(timeline);
							}
						} else {
							break;
						}
					} else {
							seqTrans.getChildren().add(parTrans);
							parTrans = new ParallelTransition();
							totRound +=1;
						}
					} 
				br.close();
				seqTrans.play();
			} catch (IOException e) {
				e.printStackTrace();
			}
		} 
	}
	
	/*
	 * This method will go through the log.txt file and get the sheperd reference such that, when playing the simulation, the sheperd will have a different color and size 
	 * (it will be always a Circle) from the other boids
	 */
	private String getSheperdReference(File f, int step) throws IOException {
		String ref = "";
		if(f.exists()) {
			try {
				BufferedReader br = new BufferedReader(new FileReader(f));
				String line;
				int counter = 0;
				while(true) {
					line = br.readLine();
					counter +=1;				 			
					if(counter == N_BOID+1) {	
						ref = line.split(",")[0].split("@")[0].split("\\.")[0];	
						break;
					}
				}
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
			
		}
		return ref;
	}

	@Override
	public void initialize(URL arg0, ResourceBundle arg1) {
		try {
            map1 = 	loadScene("view/map1.fxml");
            map2 =  loadScene("view/map2.fxml");
            map3 =  loadScene("view/map3.fxml");
            
            if(nodesMap1.isEmpty()) {
            	for (Node n : map1.getChildrenUnmodifiable()) {
					nodesMap1.add(n);
				}
    		}
            if(nodesMap2.isEmpty()) {
            	for (Node n : map2.getChildrenUnmodifiable()) {
					nodesMap2.add(n);
				}
    		}
            if(nodesMap3.isEmpty()) {
            	for (Node n : map3.getChildrenUnmodifiable()) {
					nodesMap3.add(n);
				}
    		}
        } catch (IOException ex) {
            ex.printStackTrace();
        };
    }
		
}


	
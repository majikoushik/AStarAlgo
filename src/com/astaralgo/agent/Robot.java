package com.astaralgo.agent;

import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Stack;

import com.astaralgo.environment.TileStatus;
import com.astaralgo.environment.Action;
import com.astaralgo.environment.Environment;
import com.astaralgo.environment.Position;

/**
 * Represents an intelligent agent moving through a particular room. The robot
 * only has two sensors - the ability to retrieve the the status of all its
 * neighboring tiles, including itself, and the ability to retrieve to location
 * of the TARGET tile.
 * 
 * Modify the getAction method below so that it reaches TARGET
 * with a minimal number of steps.
 */

public class Robot {
	private Environment env;
	// Creating a Stack to store all the Action with backward order
	Stack<Action> foundActions = new Stack<Action>();
	boolean aStarExecution = false;

	/** To track all the visited nodes, in terms of path **/
	Map<Node, Node> cameFrom = new HashMap<>();

	// store the actions in map
	HashMap<String, Action> actionMap = new HashMap<String, Action>();

	/** Initializes a Robot on a specific tile in the environment. */
	public Robot(Environment env) {
		this.env = env;
	}

	/**
	 * Modify the method below such that the
	 * Robot agent is able to reach the TARGET tile on a given Environment. 5 out of
	 * the 10 graded test cases, with explanations on how to create new
	 * Environments, are available under the test package.
	 * 
	 * This method should return a single Action from the Action class. -
	 * Action.DO_NOTHING - Action.MOVE_UP - Action.MOVE_DOWN - Action.MOVE_LEFT -
	 * Action.MOVE_RIGHT
	 */

	/**
	 * First the implementation starts off my calling the AStar algorithm once in
	 * the whole process to minimize
	 * the cost of calling the algorithm multiple times to figure out the best path.
	 * In the AStar algorithm, the goal is to find the optimal path with least
	 * expensive route from the starting
	 * location to the target location.
	 * Every node that is visited and added into the priority queue which sorts and
	 * gives the node with the least cost.
	 * The best path is determined by calculating the costs with every single
	 * neighboring node to understand which direction
	 * should be moved next. This direction is added to the queue to be explored
	 * later. Once the best path has been tracked,
	 * the findingPath function is responsible for backtracking from the target to
	 * the starting position. The, the getAction
	 * method picks the top action from the queue to perform that specific action.
	 * 
	 * First call of AStar function is run only during the first time. the goal of
	 * AStar function is to run once
	 * and figure out the path. get action is responsible for moving the robot.
	 */
	public Action getAction() {
		// execute only once to figure out the optimal path.
		if (!aStarExecution) {
			actionMap.put("above", Action.MOVE_UP);
			actionMap.put("below", Action.MOVE_DOWN);
			actionMap.put("left", Action.MOVE_LEFT);
			actionMap.put("right", Action.MOVE_RIGHT);
			actionMap.put("", Action.DO_NOTHING);
			AStar();
			aStarExecution = true;
		}
		else if (foundActions.size() > 0) {
			return foundActions.pop();
		} 
		return Action.DO_NOTHING;
	}

	private void AStar() {

		Position selfPosition = env.getRobotPosition(this);
		Position targetPosition = env.getTarget();

		// if starting and end position is the same - meaning target is reached
		if (selfPosition.equals(targetPosition)) {
			foundActions.add(Action.DO_NOTHING);
		}

		// Open list containing the nodes which are not explored
		PriorityQueue<Node> openList = new PriorityQueue<>();

		// creating the node from the starting position , converting the starting
		// position to the node, cost is 0 because we are starting here - no direction
		Node start = new Node(selfPosition, 0, "");

		// Adding the start node, pick up the first node at the top of the priority
		// queue
		openList.add(start);

		// store the nodes which are explored.
		Map<Position, Integer> closeList = new HashMap<>();

		// starting node is already visited so put in the bucket of nodes already
		// visited
		cameFrom.put(start, null);

		// the cost of the starting node is 0
		closeList.put(selfPosition, 0);

		// while there are more nodes to be visited meaning the open bucket has more
		// nodes to be visited
		while (!openList.isEmpty()) {
			// pick up the top node to be explored (where the priority is smaller - meaning
			// the path cost is smaller)
			Node current = openList.poll();

			// if the current node that we just explored is the target then we are done
			if (current.getPosition().equals(targetPosition)) {
				reconstructPath(current); // find the path
				break;
			}

			// get the neighboring positions of the current position
			Map<String, Position> neighboringPositions = env.getNeighborPositions(current.getPosition());
			// traverse through the neighboring four positions
			for (Map.Entry<String, Position> entry : neighboringPositions.entrySet()) {

				// Calculating the cost for the path from the starting point to the current
				// positions's neighbor node
				int new_cost = closeList.get(current.getPosition()) + 1; // step cost is 1

				// extract the position object
				Position nextPosition = entry.getValue();

				// the position picked up is not boundary and the position is not impassable and
				// the position was never visited
				if (nextPosition != null && env.getPositionTile(nextPosition).getStatus() != TileStatus.IMPASSABLE
						&& closeList.containsKey(nextPosition) == false) {

					// put the position in the visited bucket
					closeList.put(nextPosition, new_cost);
					// calculating the total cost which is f(n) = g(n) + h(n).
					int priority = new_cost + manhattan_distance_heuristic(nextPosition, targetPosition);
					// create the node object which would contain the new neighboring position, the
					// total cost, and the direction involved
					Node neighborNode = new Node(nextPosition, priority, entry.getKey());
					// add the node object to frontier bucket - so it will sit in the priority queue
					openList.add(neighborNode);
					// so add that node into the visited nodes bucket - to track the direction
					cameFrom.put(neighborNode, current);

				}

			}
		}

	}

	/**
	 * Responsible to figure out the whole path once the target node is reached
	 * through the AStar function logic
	 * 
	 * @param target
	 */
	public void reconstructPath(Node target) {
		foundActions.push(actionMap.get(target.getTileName()));
		/**
		 * Go through all the visited nodes and store it in the stack so that we back
		 * track. The stack work on LIFO
		 **/
		while (cameFrom.get(target) != null) {
			target = cameFrom.get(target);
			foundActions.push(actionMap.get(target.getTileName()));
		}
		if(foundActions.size() > 0)
		{
			//removing the first action as it will be move nothing
			foundActions.pop();
		}
	}

	/**
	 * Figure out the heuristic cost which is from current position to the target
	 * position cost using the Manhattan distance method.
	 * As the robert is not able to move diagonally, we are using Manhattan distance
	 * and not using Euclidean Distance.
	 * 
	 * @param targetPos
	 * @param nextPosition
	 * @return score
	 */
	private int manhattan_distance_heuristic(Position targetPos, Position nextPosition) {
		return Math.abs(nextPosition.getRow() - targetPos.getRow())
				+ Math.abs(nextPosition.getCol() - targetPos.getCol());
	}

	/**
	 * Node class to store the information required for the AStar logic, compares
	 * the priorities
	 */
	private class Node implements Comparable<Node> {

		private Position position;
		// score f(n) = g(n) + h(n);
		private int priority;
		private String tileName;

		public Node(Position position, int priority, String tileName) {
			this.position = position;
			this.priority = priority;
			this.tileName = tileName;
		}

		/**
		 * Extracting position from node class
		 * 
		 * @return
		 */
		public Position getPosition() {
			return position;

		}

		/**
		 * Extracting direction from node class
		 * 
		 * @return
		 */
		public String getTileName() {
			return tileName;
		}

		/**
		 * Comparing the priorities of two nodes
		 */
		@Override
		public int compareTo(Node o) {
			return Integer.compare(priority, o.priority);
		}

	}

}
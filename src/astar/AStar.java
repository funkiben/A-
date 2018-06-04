package astar;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;

/**
 * 
 * An A* implementation.
 * 
 * Can perform standard Dijkstra's Algorithm if given a constant heuristic
 * function.
 *
 * @param <T>
 *            The type of nodes the algorithm will operate on
 * 
 */
public class AStar<T extends INode<T>> {

	private final PriorityQueue<NodeData> worklist =
			new PriorityQueue<NodeData>((a, b) -> Double.compare(a.cost(), b.cost()));
	private final Map<T, NodeData> calculatedData = new HashMap<T, NodeData>();
	private final T start;
	private final ICostHeuristic<T> heuristic;

	/**
	 * 
	 * Initializes the algorithm with a starting node and the heuristic function
	 * to use.
	 * 
	 * @param start
	 *            The root node of the shortest path tree
	 * @param heuristic
	 *            A function that estimates the cost between two nodes
	 */
	public AStar(T start, ICostHeuristic<T> heuristic) {
		this.start = start;
		this.heuristic = heuristic;

	}

	/**
	 * 
	 * Gets the data for the given node that was last calculated by the last
	 * running of A*.
	 * 
	 * <p>
	 * May be null for the following reasons:
	 * 
	 * <ul>
	 * <li>the algorithm has never been run</li>
	 * <li>the node is not reachable from the starting node</li>
	 * <li>the goal was found without having the explore the given node</li>
	 * </ul>
	 * 
	 * </p>
	 * 
	 * @param node
	 *            The node whose data to retrieve
	 * @return Resulting data calculated for the node by A*
	 */
	public NodeData getData(T node) {
		return this.calculatedData.get(node);
	}

	/**
	 * 
	 * Performs A* until the given goal node is found.
	 * 
	 * <p>
	 * If the goal is null and the heuristic is a constant function, then this
	 * will generate a full shortest-path tree for all other vertices.
	 * </p>
	 * 
	 * @param goal
	 */
	public void calculate(T goal) {

		// calculate the data for the start node
		NodeData startData =
				new NodeData(start, null, 0, this.heuristic.approxCost(this.start, goal));

		this.calculatedData.clear();
		this.calculatedData.put(this.start, startData);

		this.worklist.clear();

		this.worklist.add(
				new NodeData(this.start, null, 0, this.heuristic.approxCost(this.start, goal)));

		// these variables store information about the current node being
		// visited
		NodeData cur;
		T curNode;
		double curCostFromStart;

		// this is the cost of getting to the neighbor through curNode
		double costFromStart;

		// a neighbor of curNode
		T neighbor;

		// stores the cost of the current node to neighbor
		double costFromCurNodeToNeighbor;

		// the data of neighbor
		NodeData neighborData;

		while (!this.worklist.isEmpty()) {

			cur = this.worklist.poll();
			curNode = cur.node;
			curCostFromStart = cur.costFromStart;

			if (curNode == goal) {
				break;
			}

			Collection<Edge<T>> edges = curNode.edges();

			for (Edge<T> edge : edges) {

				neighbor = edge.getTo();
				costFromCurNodeToNeighbor = edge.getCost();

				costFromStart = curCostFromStart + costFromCurNodeToNeighbor;

				neighborData = this.calculatedData.get(neighbor);

				// if neighbor has never been visited, then calculate its data
				// this includes calculating its heuristic, giving it its cost
				// from start through curNode, and setting its from to curNode
				// finally, add it to the worklist
				if (neighborData == null) {

					neighborData = new NodeData(neighbor, cur, costFromStart,
							this.heuristic.approxCost(neighbor, goal));

					this.calculatedData.put(neighbor, neighborData);
					this.worklist.add(neighborData);

				}
				// if this neighbor has been visited before, then check to see
				// if its cost from curNode is less
				// if so, then update its costFromStart and change its from node
				// to curNode
				// add it to the worklist to propagate these changes through
				// neighbors of the neighbor
				else if (costFromStart < neighborData.costFromStart) {

					neighborData.costFromStart = costFromStart;
					neighborData.from = cur;

					this.worklist.add(neighborData);

				}

			}

		}

	}

	/**
	 * 
	 * Uses data from .calculate(T) to find the path from start to the given
	 * goal. Will return an empty list if there is no path.
	 * 
	 * @param goal
	 *            The goal node where the path ends
	 * @return A list containing nodes from start to the given goal, or empty if
	 *         no such path
	 */
	public List<T> getPath(T goal) {

		List<T> path = new ArrayList<T>();

		NodeData cur = this.calculatedData.get(goal);

		while (cur != null && cur != this.start) {

			path.add(cur.node);
			cur = cur.from;

		}

		return path;

	}

	/**
	 * Stores data calculated by A* for each node. Contains the node, the node
	 * before this node in the path, the cost from start to get to this node,
	 * and the estimated distance from start to this node using the heuristic.
	 */
	public class NodeData {

		private final T node;
		private NodeData from; // may be null if no node before this one in the
								// path
		private double costFromStart;
		private final double heuristicValue;

		NodeData(T node, NodeData from, double costFromStart, double heuristicValue) {
			this.node = node;
			this.from = from;
			this.costFromStart = costFromStart;
			this.heuristicValue = heuristicValue;
		}

		/**
		 * @return The cost of moving to this node from start plus the estimated
		 *         cost from the heuristic function
		 */
		public double cost() {
			return this.costFromStart + this.heuristicValue;
		}

		/**
		 * @return The node this data is associated with
		 */
		public T getNode() {
			return this.node;
		}

		/**
		 * 
		 * @return The node before this node in the path from start to this node
		 */
		public NodeData getFrom() {
			return this.from;
		}

		/**
		 * 
		 * @return The cost to get to this node from start
		 */
		public double getCostFromStart() {
			return this.costFromStart;
		}

		/**
		 * 
		 * @return The estimated cost to get to this node from start using the
		 *         heuristic function
		 */
		public double getHeuristicValue() {
			return this.heuristicValue;
		}

	}

}

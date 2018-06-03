package astar;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.Collection;

// algorithm for computing paths
public class AStar<T extends INode<T>> {

	private final PriorityQueue<NodeData> worklist = new PriorityQueue<NodeData>(new NodeDataComparator());
	private final Map<T, NodeData> calculatedData = new HashMap<T, NodeData>();
	private final T start;
	private final ICostHeuristic<T> heuristic;

	public AStar(T start, ICostHeuristic<T> heuristic) {
		this.start = start;
		this.heuristic = heuristic;

	}
	
	// gets the data associated with the given node
	// may return null if there is none
	public NodeData getData(T node) {
		return this.calculatedData.get(node);
	}

	// runs the algorithm, calculating a path from start to the given goal
	public void calculate(T goal) {

		NodeData startData = new NodeData(start, null, 0, this.heuristic.approxCost(this.start, goal));
		
		this.calculatedData.clear();
		this.calculatedData.put(this.start, startData);

		this.worklist.clear();

		this.worklist.add(new NodeData(start, null, 0, this.heuristic.approxCost(this.start, goal)));

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
				// from start through curNode, and setting is from curNode
				// finally, add it to the worklist
				if (neighborData == null) {

					neighborData = new NodeData(neighbor, cur, costFromStart, this.heuristic.approxCost(neighbor, goal));

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

	// uses data generated to backtrack from the given goal back to start,
	// forming a path
	// may just return an empty list if the path doesn't exist
	public List<T> getPath(T goal) {

		List<T> path = new ArrayList<T>();

		NodeData cur = this.calculatedData.get(goal);

		while (cur != null && cur != this.start) {

			path.add(cur.node);
			cur = cur.from;

		}

		return path;

	}

	// wrapper class for node to store some extra info about them
	// stores: the node, the node before this node in the path, the cost from
	// start to get to this node, and the heuristic for this node
	public class NodeData {

		private final T node;
		private NodeData from; // may be null if no node before this one in the path
		private double costFromStart;
		private final double heuristicValue;

		NodeData(T node, NodeData from, double costFromStart, double heuristicValue) {
			this.node = node;
			this.from = from;
			this.costFromStart = costFromStart;
			this.heuristicValue = heuristicValue;
		}

		// total cost of moving to this node
		public double cost() {
			return this.costFromStart + this.heuristicValue;
		}

		public T getNode() {
			return this.node;
		}

		public NodeData getFrom() {
			return this.from;
		}

		public double getCostFromStart() {
			return this.costFromStart;
		}

		public double getHeuristicValue() {
			return this.heuristicValue;
		}

	}

	// class for comparing two nodes by their cost
	class NodeDataComparator implements Comparator<NodeData> {

		// gives negative if a has a lower cost than b, else positive
		@Override
		public int compare(NodeData a, NodeData b) {
			return Double.compare(a.cost(), b.cost());
		}

	}

}

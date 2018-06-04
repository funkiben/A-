package astar;

/**
 * 
 * A directional edge in a graph to be used with A*
 * 
 * @param <T> The type of nodes this edge is connecting
 */
public class Edge<T extends INode<T>> {

	private final T to;
	private final double cost;

	/**
	 * @param to The node this edge goes to
	 * @param cost The cost of traversing this edge, should be non-negative
	 */
	public Edge(T to, double cost) {
		this.to = to;
		this.cost = cost;
	}
	
	/**
	 * @return The node this edge goes to
	 */
	public T getTo() {
		return to;
	}

	/**
	 * @return The cost of traversing this edge
	 */
	public double getCost() {
		return cost;
	}

}

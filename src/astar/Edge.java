package astar;

// represents a one directional edge of a graph
public class Edge<T extends INode<T>> {

	private final T to;
	private final double cost;

	public Edge(T to, double cost) {
		this.to = to;
		this.cost = cost;
	}
	
	public T getTo() {
		return to;
	}

	public double getCost() {
		return cost;
	}

}

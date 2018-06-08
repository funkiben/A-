package example;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import astar.Edge;
import astar.INode;

// represents a node in a grid
public class GridNode implements INode<GridNode> {

	public final int x, y;
	private final List<Edge<GridNode>> connections = new ArrayList<Edge<GridNode>>();

	public GridNode(int x, int y) {
		this.x = x;
		this.y = y;
	}

	public GridNode(int x, int y, Collection<Edge<GridNode>> connections) {
		this.x = x;
		this.y = y;
		this.connections.addAll(connections);
	}

	// gets all of this gridnodes edges
	@Override
	public Collection<Edge<GridNode>> edges() {
		return this.connections;
	}

	// adds a connection from this node
	public void addConnection(GridNode to, double cost) {
		this.connections.add(new Edge<GridNode>(to, cost));
	}

	// gets the cost from this node to the given neighbor
	// returns -1 if given neighbor is not actually a neighbor
	public double getCostTo(GridNode neighbor) {

		for (Edge<GridNode> edge : this.connections) {
			if (edge.getTo() == neighbor) {
				return edge.getCost();
			}
		}

		return -1.0;
	}

	// computes a hashcode for this by just using this.x and this.y
	@Override
	public int hashCode() {
		return this.x * 1000000 + this.y;
	}

	// checks if this gridnode is the same as the given object
	// only looks at x and y coordinates
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}

		if (obj == null) {
			return false;
		}

		if (!(obj instanceof GridNode)) {
			return false;
		}

		GridNode other = (GridNode) obj;

		return this.x == other.x && this.y == other.y;
	}

}

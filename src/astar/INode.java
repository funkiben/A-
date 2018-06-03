package astar;

import java.util.Collection;

// represents a node
// generic type is the type of its neighbors
public interface INode<T extends INode<T>> {

	// list of all outgoing connections
	public Collection<Edge<T>> edges();

}

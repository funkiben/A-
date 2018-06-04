package astar;

import java.util.Collection;

/**
 * Represents a node in a graph that can be used by A*
 * 
 * @param <T> The type of node that this node connects to
 */
public interface INode<T extends INode<T>> {

	/**
	 * Gets all outgoing edges from this node.
	 * 
	 * @return A collection of edges leaving this node
	 */
	public Collection<Edge<T>> edges();

}

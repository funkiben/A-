package astar;

/**
 * A function object for estimating the cost of traveling between any two nodes.
 * 
 * @param <T>
 *            The type of nodes this heuristic acts upon
 */
public interface ICostHeuristic<T extends INode<T>> {

	/**
	 * Approximates the distance of the first node to the second. Should return
	 * a non-negative value.
	 * 
	 * @param node
	 *            The start node
	 * @param goal
	 *            The goal node
	 * @return A non-negative distance
	 */
	double approxCost(T node, T goal);

}

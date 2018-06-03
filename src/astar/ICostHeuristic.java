package astar;

// function object for heuristic of a node
public interface ICostHeuristic<T extends INode<T>> {

	// approximates distance of a node to a given goal
	// should always be positive
	double approxCost(T node, T goal);

}

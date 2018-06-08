package example;
import astar.ICostHeuristic;

public class GridHeuristic implements ICostHeuristic<GridNode> {

	private final GridNode[][] grid;
	private final double costWeight, distWeight;

	public GridHeuristic(GridNode[][] grid, double costWeight, double distWeight) {
		this.grid = grid;
		this.costWeight = costWeight;
		this.distWeight = distWeight;
	}

	// approximates cost from a to b
	// sums up the weights of nodes from a to b in straight line
	@Override
	public double approxCost(GridNode a, GridNode b) {

		double approxCost = 0.0;

		int dx = b.x - a.x;
		int dy = b.y - a.y;

		if (Math.abs(dx) > Math.abs(dy)) {

			int change = a.x < b.x ? 1 : -1;
			double slope = (double) dy / dx;
			double intercept = a.y - slope * a.x;

			GridNode next, cur = a;

			for (int x = a.x; x != b.x; x += change) {

				next = this.grid[x + change][(int) (slope * (x + change) + intercept)];

				approxCost += cur.getCostTo(next);

				cur = next;

			}

		} else {

			int change = a.y < b.y ? 1 : -1;
			double slope = (double) dx / dy;
			double intercept = a.x - slope * a.y;

			GridNode next, cur = a;

			for (int y = a.y; y != b.y; y += change) {

				next = this.grid[(int) (slope * (y + change) + intercept)][y + change];

				approxCost += cur.getCostTo(next);

				cur = next;

			}

		}

		return costWeight * approxCost + distWeight * Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

	}
}

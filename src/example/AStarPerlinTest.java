package example;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;

import astar.AStar;
import astar.ICostHeuristic;
import example.noise.PerlinNoiseGenerator;

public class AStarPerlinTest extends JFrame implements MouseMotionListener {

	public static void main(String[] args) {
		new AStarPerlinTest();
	}

	private static final long serialVersionUID = 1L;

	private class DrawCanvas extends JPanel {

		private static final long serialVersionUID = 1L;

		@Override
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			setBackground(Color.WHITE);
			draw(g);
		}

	}

	private int size = 800;
	private int tiles = 200;
	private double heightWeight = 50;
	private GridNode[][] grid;
	private double[][] heights;
	private AStar<GridNode> astar;

	private int octaves = 3;
	private double amplitude = 5;
	private double frequency = 0.35;

	int goalX = 100;
	int goalY = 100;

	private BufferedImage perlinImg = new BufferedImage(tiles, tiles, BufferedImage.TYPE_INT_RGB);

	public AStarPerlinTest() {

		PerlinNoiseGenerator noise = new PerlinNoiseGenerator(new Random());

		this.heights = new double[tiles][tiles];

		this.grid = new GridNode[tiles][tiles];

		for (int col = 0; col < tiles; col++) {
			for (int row = 0; row < tiles; row++) {

				grid[col][row] = new GridNode(col, row);
				heights[col][row] =
						Math.abs(noise.noise(col, row, octaves, frequency, amplitude, true));

				int c = (int) (heights[col][row] * 255);
				perlinImg.setRGB(col, row, new Color(c, c, c).getRGB());

			}
		}

		double root2 = Math.sqrt(2.0); // diagonal distance between two tiles

		for (int col = 0; col < tiles; col++) {
			for (int row = 0; row < tiles; row++) {

				GridNode node = grid[col][row];

				boolean colPlusOne = col + 1 < tiles;
				boolean colMinusOne = col - 1 >= 0;
				boolean rowPlusOne = row + 1 < tiles;
				boolean rowMinusOne = row - 1 >= 0;

				double nodeHeight = heights[col][row];

				if (colPlusOne) {
					node.addConnection(grid[col + 1][row],
							Math.abs(nodeHeight - heights[col + 1][row]) * heightWeight + 1);
				}

				if (rowPlusOne) {
					node.addConnection(grid[col][row + 1],
							Math.abs(nodeHeight - heights[col][row + 1]) * heightWeight + 1);
				}

				if (colMinusOne) {
					node.addConnection(grid[col - 1][row],
							Math.abs(nodeHeight - heights[col - 1][row]) * heightWeight + 1);
				}

				if (rowMinusOne) {
					node.addConnection(grid[col][row - 1],
							Math.abs(nodeHeight - heights[col][row - 1]) * heightWeight + 1);
				}

				if (colPlusOne && rowPlusOne) {
					node.addConnection(grid[col + 1][row + 1],
							Math.abs(nodeHeight - heights[col + 1][row + 1]) * heightWeight
									+ root2);
				}

				if (colMinusOne && rowPlusOne) {
					node.addConnection(grid[col - 1][row + 1],
							Math.abs(nodeHeight - heights[col - 1][row + 1]) * heightWeight
									+ root2);
				}

				if (colMinusOne && rowMinusOne) {
					node.addConnection(grid[col - 1][row - 1],
							Math.abs(nodeHeight - heights[col - 1][row - 1]) * heightWeight
									+ root2);
				}

				if (colPlusOne && rowMinusOne) {
					node.addConnection(grid[col + 1][row - 1],
							Math.abs(nodeHeight - heights[col + 1][row - 1]) * heightWeight
									+ root2);
				}

			}
		}

		ICostHeuristic<GridNode> heuristic = new GridHeuristic(this.grid, 0.5, 0.5);

		astar = new AStar<GridNode>(grid[tiles / 2][tiles / 2], heuristic);

		this.addMouseMotionListener(this);

		setSize(size, size);
		add(new DrawCanvas());
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setResizable(false);
		setVisible(true);

	}

	void draw(Graphics g) {

		g.drawImage(perlinImg, 0, 0, this.size, this.size, null);

		drawPath(g, Color.RED, this.astar);

	}

	void drawPath(Graphics g, Color color, AStar<GridNode> astar) {
		int tileSize = this.size / this.tiles;

		List<GridNode> path = astar.getPath(this.grid[goalX][goalY]);

		g.setColor(color);

		GridNode prev = null;
		for (GridNode node : path) {
			if (prev != null) {
				g.drawLine(node.x * tileSize + tileSize / 2, node.y * tileSize + tileSize / 2,
						prev.x * tileSize + tileSize / 2, prev.y * tileSize + tileSize / 2);
			}
			prev = node;
		}
	}

	void drawVisitedNodes(AStar<GridNode> astar) {
		double maxCost = Double.MIN_VALUE;
		double minCost = Double.MAX_VALUE;

		for (int col = 0; col < tiles; col++) {
			for (int row = 0; row < tiles; row++) {

				AStar<GridNode>.NodeData data = astar.getData(this.grid[col][row]);

				if (data != null) {

					if (data.cost() > maxCost) {
						maxCost = data.cost();
					}

					if (data.cost() < minCost) {
						minCost = data.cost();
					}

				}

			}
		}

		double range = maxCost - minCost;

		for (int col = 0; col < tiles; col++) {
			for (int row = 0; row < tiles; row++) {

				int c = (int) (heights[col][row] * 255);

				AStar<GridNode>.NodeData data = astar.getData(this.grid[col][row]);

				if (data != null) {

					perlinImg.setRGB(col, row,
							new Color(0,
									(int) ((data.cost() - minCost) / range * 255.0 / 3) + c / 3, 0)
											.getRGB());

				} else {

					perlinImg.setRGB(col, row, new Color(c, c, c).getRGB());

				}

			}
		}
	}

	@Override
	public void mouseMoved(MouseEvent e) {

		this.goalX = (int) Math.max(Math.min(e.getX() / ((double) size / tiles), tiles - 1), 0);
		this.goalY = (int) Math.max(Math.min(e.getY() / ((double) size / tiles), tiles - 1), 0);

		this.astar.calculate(this.grid[goalX][goalY]);

		this.drawVisitedNodes(this.astar);

		this.repaint();
	}

	@Override
	public void mouseDragged(MouseEvent e) {
	}

}

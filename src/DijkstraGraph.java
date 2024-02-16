// --== CS400 File Header Information ==--
// Name: Muhammad Irfan Bin Mohd Ropi
// Email: binmohdropi@wisc.edu
// Group and Team: E04
// Group TA: Lakshika Rathi
// Lecturer: Gary Dahl

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.*;

/**
 * This class extends the BaseGraph data structure with additional methods for computing the total
 * cost and list of node data along the shortest path connecting a provided starting to ending
 * nodes. This class makes use of Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number> extends BaseGraph<NodeType, EdgeType>
    implements GraphADT<NodeType, EdgeType> {

  /**
   * While searching for the shortest path between two nodes, a SearchNode contains data about one
   * specific path between the start node and another node in the graph. The final node in this path
   * is stored in its node field. The total cost of this path is stored in its cost field. And the
   * predecessor SearchNode within this path is referened by the predecessor field (this field is
   * null within the SearchNode containing the starting node in its node field).
   *
   * SearchNodes are Comparable and are sorted by cost so that the lowest cost SearchNode has the
   * highest priority within a java.util.PriorityQueue.
   */
  protected class SearchNode implements Comparable<SearchNode> {
    public Node node;
    public double cost;
    public SearchNode predecessor;

    public SearchNode(Node node, double cost, SearchNode predecessor) {
      this.node = node;
      this.cost = cost;
      this.predecessor = predecessor;
    }

    public int compareTo(SearchNode other) {
      if (cost > other.cost)
        return +1;
      if (cost < other.cost)
        return -1;
      return 0;
    }
  }

  /**
   * Constructor that sets the map that the graph uses.
   *
   * @param map the map that the graph uses to map a data object to the node object it is stored in
   */
  public DijkstraGraph(MapADT<NodeType, Node> map) {
    super(map);
  }

  /**
   * This helper method creates a network of SearchNodes while computing the shortest path between
   * the provided start and end locations. The SearchNode that is returned by this method is
   * represents the end of the shortest path that is found: it's cost is the cost of that shortest
   * path, and the nodes linked together through predecessor references represent all of the nodes
   * along that shortest path (ordered from end to start).
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return SearchNode for the final end node within the shortest path
   * @throws NoSuchElementException when no path from start to end is found or when either start or
   *                                end data do not correspond to a graph node
   */
  protected SearchNode computeShortestPath(NodeType start, NodeType end) {
    // implement in step 5.3

    // check if start and end exist in the graph
    if (!nodes.containsKey(start)) {
      throw new NoSuchElementException("No such nodes as " + start.toString() + " in the graph");
    }
    if (!nodes.containsKey(end)) {
      throw new NoSuchElementException("No such nodes as " + end.toString() + " in the graph");
    }

    // create a priority queue of SearchNodes
    PriorityQueue<SearchNode> pathQueue = new PriorityQueue<>();

    // create a search node for the start node
    SearchNode startSearchNode = new SearchNode(nodes.get(start), 0.0, null);

    // create a map of visited nodes
    MapADT<NodeType, Node> visited = new PlaceholderMap<>();

    // Add the start node to the priority queue
    pathQueue.add(startSearchNode);

    // while the queue is not empty, find the shortest path
    while (!pathQueue.isEmpty()) {
      // remove the most recent node from the queue
      SearchNode current = pathQueue.remove();

      //if the current node had been visited, skip it
      if (visited.containsKey(current.node.data)) {
        continue;
      }

      // if we reached the end node, return the current node
      if (current.node.data.equals(end)) {
        // add the current node to the visited map
        visited.put(current.node.data, current.node);
        return current;
      }

      // else, add the current node to the visited map
      visited.put(current.node.data, current.node);

      // Then we add all the neighbors of the current node to the queue
      for (Edge edge : current.node.edgesLeaving){
        // check if the neighbor that we are adding has been visited or not
        if(!visited.containsKey(edge.successor.data)) {
          // if not, add it to the queue
          pathQueue.add(
              new SearchNode(edge.successor, edge.data.doubleValue() + current.cost, current));
        }
      }
    }

    // if we reach this point, no path was found
    throw new NoSuchElementException("No path found between " + start.toString() + " and " + end.toString());
  }

  /**
   * Returns the list of data values from nodes along the shortest path from the node with the
   * provided start value through the node with the provided end value. This list of data values
   * starts with the start value, ends with the end value, and contains intermediary values in the
   * order they are encountered while traversing this shorteset path. This method uses Dijkstra's
   * shortest path algorithm to find this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return list of data item from node along this shortest path
   */
  public List<NodeType> shortestPathData(NodeType start, NodeType end) {
    if (!nodes.containsKey(start)) {
      throw new NoSuchElementException("No such nodes as " + start.toString() + " in the graph");
    }
    if (!nodes.containsKey(end)) {
      throw new NoSuchElementException("No such nodes as " + end.toString() + " in the graph");
    }

    // initialize the search node to backtrack
    SearchNode pathNodeData = computeShortestPath(start, end);

    // create a list to store the data
    List<NodeType> path = new LinkedList<>();

    //pushing the data into the list from front
    while (pathNodeData != null) {
      path.add(0, pathNodeData.node.data);
      pathNodeData = pathNodeData.predecessor;
    }

    return path;
  }

  /**
   * Returns the cost of the path (sum over edge weights) of the shortest path freom the node
   * containing the start data to the node containing the end data. This method uses Dijkstra's
   * shortest path algorithm to find this solution.
   *
   * @param start the data item in the starting node for the path
   * @param end   the data item in the destination node for the path
   * @return the cost of the shortest path between these nodes
   */
  public double shortestPathCost(NodeType start, NodeType end) {
    if (!nodes.containsKey(start)) {
      throw new NoSuchElementException("No such nodes as " + start.toString() + " in the graph");
    }
    if (!nodes.containsKey(end)) {
      throw new NoSuchElementException("No such nodes as " + end.toString() + " in the graph");
    }

    // initialize the search node to backtrack
    SearchNode pathNodeCost = computeShortestPath(start, end);
    return pathNodeCost.cost;
  }

  // TODO: implement 3+ tests in step 4.1



  /**
   * Tests the shortestPathData method. It tests the data along the shortest path is correct.
   */
  @Test
  public void testShortestPathData() {

    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // Adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("L");
    graph.insertNode("M");

    //adding edges
    graph.insertEdge("A", "B", 1);
    graph.insertEdge("A", "H", 8);
    graph.insertEdge("A", "M", 5);
    graph.insertEdge("B", "M", 3);
    graph.insertEdge("D", "A", 7);
    graph.insertEdge("D", "G", 2);
    graph.insertEdge("F", "G", 9);
    graph.insertEdge("G", "L", 7);
    graph.insertEdge("H", "B", 6);
    graph.insertEdge("H", "I", 2);
    graph.insertEdge("I", "D", 1);
    graph.insertEdge("I", "L", 5);
    graph.insertEdge("I", "H", 2);
    graph.insertEdge("M", "E", 3);
    graph.insertEdge("M", "F", 4);

    // actual path data that is returned
    List<String> actualPathData = graph.shortestPathData("D", "I");

    // expected path data that should be returned
    List<String> expectedPathData = new LinkedList<>();
    expectedPathData.add(0, "I");
    expectedPathData.add(0, "H");
    expectedPathData.add(0, "A");
    expectedPathData.add(0, "D");

    // Checking each element in actualDataPath match the element in expectedDataPath
    int i = 0;
    for (String data : actualPathData) {
      Assertions.assertEquals(data, expectedPathData.get(i));
      i++;
    }

    // implement in step 4.1
  }

  /**
   * Tests the shortestPathCost method. It tests the cost of the shortest path is correct.
   */
  @Test
  public void testShortestPathCost() {
    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("L");
    graph.insertNode("M");

    // adding edges
    graph.insertEdge("A", "B", 1);
    graph.insertEdge("A", "H", 8);
    graph.insertEdge("A", "M", 5);
    graph.insertEdge("B", "M", 3);
    graph.insertEdge("D", "A", 7);
    graph.insertEdge("D", "G", 2);
    graph.insertEdge("F", "G", 9);
    graph.insertEdge("G", "L", 7);
    graph.insertEdge("H", "B", 6);
    graph.insertEdge("H", "I", 2);
    graph.insertEdge("I", "D", 1);
    graph.insertEdge("I", "L", 5);
    graph.insertEdge("I", "H", 2);
    graph.insertEdge("M", "E", 3);
    graph.insertEdge("M", "F", 4);

    // actual path data that is returned
    double actualPathCost = graph.shortestPathCost("D", "I");

    // expected path data that should be returned
    double expectedPathCost = 17.0;

    // Checking each element in actualDataPath match the element in expectedDataPath
    Assertions.assertEquals(expectedPathCost, actualPathCost);
  }

  /**
   * Tests the shortestPathData method. It validates that when
   * two nodes that exist but no directed path exist between them, it throws a NoSuchElementException.
   */
  @Test
  public void testNoDirectedPathExist() {
    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("L");
    graph.insertNode("M");

    // adding edges
    graph.insertEdge("A", "B", 1);
    graph.insertEdge("A", "H", 8);
    graph.insertEdge("A", "M", 5);
    graph.insertEdge("B", "M", 3);
    graph.insertEdge("D", "A", 7);
    graph.insertEdge("D", "G", 2);
    graph.insertEdge("F", "G", 9);
    graph.insertEdge("G", "L", 7);
    graph.insertEdge("H", "B", 6);
    graph.insertEdge("H", "I", 2);
    graph.insertEdge("I", "D", 1);
    graph.insertEdge("I", "L", 5);
    graph.insertEdge("I", "H", 2);
    graph.insertEdge("M", "E", 3);
    graph.insertEdge("M", "F", 4);

    Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData("L", "E"));

  }

  /**
   * Tests the shortestPathData method. It validates that the
   * traversal of the shortest path between start and end nodes is correct.
   */
  @Test
  public void testSearchNodePath() {
    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("L");
    graph.insertNode("M");

    // adding edges
    graph.insertEdge("A", "B", 1);
    graph.insertEdge("A", "H", 8);
    graph.insertEdge("A", "M", 5);
    graph.insertEdge("B", "M", 3);
    graph.insertEdge("D", "A", 7);
    graph.insertEdge("D", "G", 2);
    graph.insertEdge("F", "G", 9);
    graph.insertEdge("G", "L", 7);
    graph.insertEdge("H", "B", 6);
    graph.insertEdge("H", "I", 2);
    graph.insertEdge("I", "D", 1);
    graph.insertEdge("I", "L", 5);
    graph.insertEdge("I", "H", 2);
    graph.insertEdge("M", "E", 3);
    graph.insertEdge("M", "F", 4);

    SearchNode current = (SearchNode) graph.computeShortestPath("D", "I");
    List<String> pathData = new LinkedList<>();
    while (current != null) {
      pathData.add(0, (String) current.node.data);
      current = current.predecessor;
    }
    Assertions.assertEquals("[D, A, H, I]", pathData.toString());
  }

  /**
   * Tests the shortestPathData method. It validates that the
   * cost of the shortest path between start and end nodes is correct.
   */
  @Test
  public void testSearchNodeCost() {
    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("L");
    graph.insertNode("M");

    //  adding edges
    graph.insertEdge("A", "B", 1);
    graph.insertEdge("A", "H", 8);
    graph.insertEdge("A", "M", 5);
    graph.insertEdge("B", "M", 3);
    graph.insertEdge("D", "A", 7);
    graph.insertEdge("D", "G", 2);
    graph.insertEdge("F", "G", 9);
    graph.insertEdge("G", "L", 7);
    graph.insertEdge("H", "B", 6);
    graph.insertEdge("H", "I", 2);
    graph.insertEdge("I", "D", 1);
    graph.insertEdge("I", "L", 5);
    graph.insertEdge("I", "H", 2);
    graph.insertEdge("M", "E", 3);
    graph.insertEdge("M", "F", 4);

    // compute the shortest path
    SearchNode current = (SearchNode) graph.computeShortestPath("D", "I");
    List<Double> pathData = new LinkedList<>();
    while (current != null) {
      pathData.add(0, current.cost);
      current = current.predecessor;
    }
    Assertions.assertEquals("[0.0, 7.0, 15.0, 17.0]", pathData.toString());
  }

  /**
   * Tests the shortestPathCost and shortestPathData method. It tests the cost of the shortest path
   * is correct and the shortest path data is correct.
   */
  @Test
  public void testComplexGraphShortestPath() {
    // create a graph with 4 nodes and 5 edges
    DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());

    // adding nodes
    graph.insertNode("A");
    graph.insertNode("B");
    graph.insertNode("C");
    graph.insertNode("D");
    graph.insertNode("E");
    graph.insertNode("F");
    graph.insertNode("G");
    graph.insertNode("H");
    graph.insertNode("I");
    graph.insertNode("J");
    graph.insertNode("K");
    graph.insertNode("L");

    // adding edges
    graph.insertEdge("A" , "B" ,3);
    graph.insertEdge("A" , "F" ,5);
    graph.insertEdge("A" , "I" ,8);
    graph.insertEdge("B" , "C" ,5);
    graph.insertEdge("B" , "D" ,1);
    graph.insertEdge("B" , "G" ,2);
    graph.insertEdge("B" , "I" ,3);
    graph.insertEdge("C" , "D" ,2);
    graph.insertEdge("C" , "E" ,7);
    graph.insertEdge("C" , "H" ,6);
    graph.insertEdge("C" , "J" ,2);
    graph.insertEdge("D" , "E" ,4);
    graph.insertEdge("D" , "G" ,3);
    graph.insertEdge("D" , "K" ,5);
    graph.insertEdge("E" , "A" ,6);
    graph.insertEdge("E" , "F" ,1);
    graph.insertEdge("E" , "H" ,2);
    graph.insertEdge("E" , "L" ,9);
    graph.insertEdge("F" , "G" ,2);
    graph.insertEdge("F" , "J" ,1);
    graph.insertEdge("G" , "H" ,3);
    graph.insertEdge("G" , "K" ,3);
    graph.insertEdge("H" , "F" ,4);
    graph.insertEdge("H" , "L" ,2);
    graph.insertEdge("I" , "C" ,7);
    graph.insertEdge("I" , "F" ,4);
    graph.insertEdge("I" , "J" ,5);
    graph.insertEdge("J" , "D" ,2);
    graph.insertEdge("J" , "G" ,6);
    graph.insertEdge("J" , "K" ,4);
    graph.insertEdge("K" , "E" ,8);
    graph.insertEdge("K" , "H" ,5);
    graph.insertEdge("K" , "L" ,6);
    graph.insertEdge("L" , "A" ,7);
    graph.insertEdge("L" , "B" ,3);

    //Checking the cost and data of the shortest path for several path
    Assertions.assertEquals(10.0, graph.shortestPathCost("A", "L"));
    Assertions.assertEquals("[A, B, G, H, L]", graph.shortestPathData("A", "L").toString());

    Assertions.assertEquals(3.0, graph.shortestPathCost("E", "G"));
    Assertions.assertEquals("[E, F, G]", graph.shortestPathData("E", "G").toString());

    Assertions.assertEquals(8.0, graph.shortestPathCost("L", "C"));
    Assertions.assertEquals("[L, B, C]", graph.shortestPathData("L", "C").toString());

    Assertions.assertEquals(10.0, graph.shortestPathCost("F", "B"));
    Assertions.assertEquals("[F, G, H, L, B]", graph.shortestPathData("F", "B").toString());

    Assertions.assertEquals(9.0, graph.shortestPathCost("K", "F"));
    Assertions.assertEquals("[K, E, F]", graph.shortestPathData("K", "F").toString());

  }
}

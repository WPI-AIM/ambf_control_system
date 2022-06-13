// C / C++ program for Dijkstra's 
// shortest path algorithm for adjacency
// list representation of graph
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "rbdl_model/ModelStructs.h"
#include "rbdl_model/GraphEdge.h"
#include "rbdl_model/MinHeapStructs.h"
#include "rbdl_model/MinHeap.h"
#include <set>
#include <vector>

class ModelGraph
{
  public:

  ModelGraph(GraphEdge edges[], int V, int n);
  ~ModelGraph();
  void CreateGraph(struct Graph** graph);
  const int BaseNode();
  std::vector<int> EndEffectorNodes();
  std::vector<int> ShortestPath(int src, int dest);

  private:
  struct AdjListNode* NewAdjListNode(int dest, int weight);
  void AddEdge(struct Graph** graph, struct GraphEdge* edge);
  void PrintAdjacencyMatrix();
  void PrintArr(int dist[], int n);  
  void Dijkstra(int src);
  void CleanUp(struct Graph** graph);
  
  private:
  int V_; // number of nodes in the graph
  int E_; // numver of edges in the graph
  struct Graph* graph_;
  bool hasRanDijkstra{false};
};
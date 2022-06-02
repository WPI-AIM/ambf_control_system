#include "rbdl_model/ModelGraph.h"
//Ref: https://www.geeksforgeeks.org/dijkstras-algorithm-for-adjacency-list-representation-greedy-algo-8/
ModelGraph::ModelGraph(GraphEdge edges[], int V, int n)
{
  this->V_ = V;

  CreateGraph();

  for (int i = 0; i < n; ++i)
    AddEdge(&edges[i]);
}

// A utility function that creates 
// a graph of V vertices
void ModelGraph::CreateGraph()
{
  graph_ = (struct Graph*) malloc(sizeof(struct Graph));
  graph_->V = V_;

  // Create an array of adjacency lists.  
  // Size of array will be V
  graph_->array = (struct AdjList*) malloc(V_ * sizeof(struct AdjList));

  // Initialize each adjacency list 
  // as empty by making head as NULL
  for (int i = 0; i < V_; ++i)
    graph_->array[i].head = NULL;
}


// A utility function to create 
// a new adjacency list node
struct AdjListNode* ModelGraph::NewAdjListNode(int dest, int weight)
{
  struct AdjListNode* newNode = 
    (struct AdjListNode*) malloc(sizeof(struct AdjListNode));
  newNode->dest = dest;
  newNode->weight = weight;
  newNode->next = NULL;
  return newNode;
}

// Adds an edge to an undirected graph
void ModelGraph::AddEdge(struct GraphEdge* edge)
{
  // Add an edge from src to dest.  
  // A new node is added to the adjacency
  // list of src.  The node is 
  // added at the beginning
  struct AdjListNode* newNode = NewAdjListNode(edge->dest, edge->weight);
  newNode->next = graph_->array[edge->src].head;
  graph_->array[edge->src].head = newNode;

  // Since graph is undirected, 
  // add an edge from dest to src also
  newNode = NewAdjListNode(edge->src, edge->weight);
  newNode->next = graph_->array[edge->dest].head;
  graph_->array[edge->dest].head = newNode;
}


// A utility function used to print the solution
void ModelGraph::PrintArr(int dist[], int n)
{
  printf("Vertex   Distance from Source\n");
  for (int i = 0; i < n; ++i)
    printf("%d \t\t %d\n", i, dist[i]);
}
  
// The main function that calculates 
// distances of shortest paths from src to all
// vertices. It is a O(ELogV) function
void ModelGraph::Dijkstra(int src)
{ 
  MinHeap minHeap;
  // Get the number of vertices in graph
  int V = graph_->V;
  
  // dist values used to pick
  // minimum weight edge in cut
  int dist[V];     

  // minHeap represents set E
  struct MinHeapStruct* minHeapStruct = minHeap.CreateMinHeap(V);

  // Initialize min heap with all 
  // vertices. dist value of all vertices 
  for (int v = 0; v < V; ++v)
  {
      dist[v] = INT_MAX;
      minHeapStruct->array[v] = minHeap.NewMinHeapNode(v, 
                                    dist[v]);
      minHeapStruct->pos[v] = v;
  }

  // Make dist value of src vertex 
  // as 0 so that it is extracted first
  minHeapStruct->array[src] = minHeap.NewMinHeapNode(src, dist[src]);
  minHeapStruct->pos[src] = src;
  dist[src] = 0;
  minHeap.DecreaseKey(minHeapStruct, src, dist[src]);

  // Initially size of min heap is equal to V
  minHeapStruct->size = V;

  // In the followin loop, 
  // min heap contains all nodes
  // whose shortest distance 
  // is not yet finalized.
  while (!minHeap.IsEmpty(minHeapStruct))
  {
    // Extract the vertex with 
    // minimum distance value
    struct MinHeapNode* minHeapNode = minHeap.ExtractMin(minHeapStruct);
    
    // Store the extracted vertex number
    int u = minHeapNode->v; 

    // Traverse through all adjacent 
    // vertices of u (the extracted
    // vertex) and update 
    // their distance values
    struct AdjListNode* pCrawl = graph_->array[u].head;
    while (pCrawl != NULL)
    {
      int v = pCrawl->dest;

      // If shortest distance to v is
      // not finalized yet, and distance to v
      // through u is less than its 
      // previously calculated distance
      if (minHeap.IsInMinHeap(minHeapStruct, v) && 
        dist[u] != INT_MAX && 
        pCrawl->weight + dist[u] < dist[v])
      {
        dist[v] = dist[u] + pCrawl->weight;

        // update distance 
        // value in min heap also
        minHeap.DecreaseKey(minHeapStruct, v, dist[v]);
      }
      pCrawl = pCrawl->next;
    }
  }
  minHeap.~MinHeap();
  // print the calculated shortest distances
  PrintArr(dist, V);
}

ModelGraph::~ModelGraph()
{
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    // printf("array.head->dest %d\n", array.head->dest);
    while(array.head != nullptr)
    {
      // printf("%d, ", array.head->dest);
      struct AdjListNode *tempNode = array.head;
      array.head = array.head->next;

      delete tempNode;
    }
    // printf("\n");
  }

  free(graph_->array);
  printf("Cleanded ModelGraph Memory\n");
}
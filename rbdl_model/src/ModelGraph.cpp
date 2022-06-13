#include "rbdl_model/ModelGraph.h"
ModelGraph::ModelGraph(GraphEdge edges[], int V, int E)
{
  this->V_ = V;
  this->E_ = E;
  CreateGraph(&graph_);

  for (int i = 0; i < E; ++i)
    AddEdge(&graph_, &edges[i]);
}

// A utility function that creates 
// a graph of V vertices
void ModelGraph::CreateGraph(struct Graph** graph)
{
  *graph = (struct Graph*) malloc(sizeof(struct Graph));
  (*graph)->V = V_;

  // Create an array of adjacency lists.  
  // Size of array will be V
  (*graph)->array = (struct AdjList*) malloc(V_ * sizeof(struct AdjList));

  // Initialize each adjacency list 
  // as empty by making head as NULL
  for (int i = 0; i < V_; ++i)
    (*graph)->array[i].head = NULL;
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
void ModelGraph::AddEdge(struct Graph** graph, struct GraphEdge* edge)
{
  // Add an edge from src to dest.  
  // A new node is added to the adjacency
  // list of src.  The node is 
  // added at the beginning
  struct AdjListNode* newNode = NewAdjListNode(edge->dest, edge->weight);
  newNode->next = (*graph)->array[edge->src].head;
  (*graph)->array[edge->src].head = newNode;

  // Since graph is undirected, 
  // add an edge from dest to src also
  // newNode = NewAdjListNode(edge->src, edge->weight);
  // newNode->next = graph_->array[edge->dest].head;
  // graph_->array[edge->dest].head = newNode;
}

void ModelGraph::PrintAdjacencyMatrix()
{
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    printf("parent: %d------->", v);
    while(array.head != nullptr)
    {
      printf("%d, ", array.head->dest);
      struct AdjListNode *tempNode = array.head;
      array.head = array.head->next;
    }
    printf("\n");
  }
}

const int ModelGraph::BaseNode()
{
  std::set<int> indices;
  for (int i = 0; i < V_; ++i)
  {
    indices.insert(indices.end(), i);
  }

  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    while(array.head != nullptr)
    {
      indices.erase(array.head->dest);
      array.head = array.head->next;
    }
  }

  return (indices.size() == 1 ? *(indices.begin()) : -1);
}

std::vector<int> ModelGraph::EndEffectorNodes()
{
  std::vector<int> endEffectorNodes;

  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    // Endeffector do not have child. Few models like PSM has multiple End effectors.
    if(array.head == nullptr) endEffectorNodes.emplace_back(v);
  }

  return endEffectorNodes;
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
  // dist values used to pick
  // minimum weight edge in cut
  int dist[V_];

  MinHeap minHeap;
  // Get the number of vertices in graph
  int V = graph_->V;
  
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
  // PrintArr(dist, V);
  hasRanDijkstra = true;
}

std::vector<int> ModelGraph::ShortestPath(int src, int dest)
{
  if(!hasRanDijkstra) this->Dijkstra(src);

  int pathSequence[V_];  
  GraphEdge destToSrcEdges[E_];

  int e = 0;
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    while(array.head != nullptr)
    {
      destToSrcEdges[e] = { array.head->dest, v, array.head->weight };
      array.head = array.head->next;
      e++;
    }
  }
  struct Graph* destToSrcGraph;
  CreateGraph(&destToSrcGraph);
  for (int i = 0; i < E_; ++i)
    AddEdge(&destToSrcGraph, &destToSrcEdges[i]);
  
  std::vector<int> path;

  MinHeap minHeap;
  // minHeap represents set E
  struct MinHeapStruct* minHeapStruct = minHeap.CreateMinHeap(V_);

  // Get the number of vertices in graph
  int V = graph_->V;
  
  // dist values used to pick
  // minimum weight edge in cut
  int distFromDest[V];     


  // Initialize min heap with all 
  // vertices. dist value of all vertices 
  for (int v = 0; v < V; ++v)
  {
      distFromDest[v] = INT_MAX;
      minHeapStruct->array[v] = minHeap.NewMinHeapNode(v, distFromDest[v]);
      minHeapStruct->pos[v] = v;
  }

  // Make dist value of dest vertex 
  // as 0 so that it is extracted first
  minHeapStruct->array[dest] = minHeap.NewMinHeapNode(dest, distFromDest[dest]);
  minHeapStruct->pos[dest] = dest;
  distFromDest[dest] = 0;
  minHeap.DecreaseKey(minHeapStruct, dest, distFromDest[dest]);

  // // Initially size of min heap is equal to V
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


    // Traverse through all vertices thats leads to u
    // and update their distance values
    struct AdjListNode* pCrawl = destToSrcGraph-> array[u].head;
    while (pCrawl != NULL)
    {
      int v = pCrawl->dest;
      // If shortest distance to v is
      // not finalized yet, and distance to v
      // through u is less than its 
      // previously calculated distance
      if (minHeap.IsInMinHeap(minHeapStruct, v) &&  
        pCrawl->weight + distFromDest[u] < distFromDest[v])
      {
        distFromDest[v] = distFromDest[u] + pCrawl->weight;
        pathSequence[v] = u;
        // update distance 
        // value in min heap also
        minHeap.DecreaseKey(minHeapStruct, v, distFromDest[v]);
      }
      pCrawl = pCrawl->next;
    }
  }
  minHeap.~MinHeap();

  int v = src;
  path.emplace_back(v);
  while(v != dest)
  {
    path.emplace_back(pathSequence[v]);
    v = pathSequence[v];
  }

  CleanUp(&destToSrcGraph);
  return path;
}


void ModelGraph::CleanUp(struct Graph** graph)
{
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = (*graph)->array[v];
    // printf("parent: %d\n", v);
    while(array.head != nullptr)
    {
      // printf("%d, ", array.head->dest);
      struct AdjListNode *tempNode = array.head;
      array.head = array.head->next;

      delete tempNode;
    }
  }
  free((*graph)->array);
}

ModelGraph::~ModelGraph()
{
  CleanUp(&graph_);
  printf("Cleanded ModelGraph Memory\n");
}
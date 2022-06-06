#include "rbdl_model/ModelGraph.h"
//Ref: https://www.geeksforgeeks.org/dijkstras-algorithm-for-adjacency-list-representation-greedy-algo-8/
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
void ModelGraph::Dijkstra(int src, int dist[])
{ 
  MinHeap minHeap;
  // Get the number of vertices in graph
  int V = graph_->V;
  
  // dist values used to pick
  // minimum weight edge in cut
  // int dist[V];     

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
}

std::vector<int> ModelGraph::ShortestPath(int src, int dest)
{
  int distFromSrc[V_];
  int pathSequence[V_];
  this->Dijkstra(src, distFromSrc);
  
  GraphEdge destToSrcEdges[E_];

  int e = 0;
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    // printf("parent: %d------->", v);
    while(array.head != nullptr)
    {
      // printf("%d, ", array.head->dest);
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

  for(int e = 0; e < E_; e++)
  {
    printf("{ %d, %d, %d }\n", destToSrcEdges[e].src, destToSrcEdges[e].dest, destToSrcEdges[e].weight);
  }

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

  printf("distFromDest\n");
  for(int i = 0; i < V; i++)
    printf("i: %d, distFromDest[i]:%d \n", i, distFromDest[i]);
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

    printf("u: %d-> \n", u);
    struct AdjListNode* pCrawl = destToSrcGraph-> array[u].head;
    while (pCrawl != NULL)
    {
      int v = pCrawl->dest;
      printf("distFromDest[%d]:%d + pCrawl->weight: %d < distFromDest[%d]: %d\n", 
        u, distFromDest[u], pCrawl->weight, v, distFromDest[v]);
      // pathSequence[u] = v;
      // If shortest distance to v is
      // not finalized yet, and distance to v
      // through u is less than its 
      // previously calculated distance
      if (minHeap.IsInMinHeap(minHeapStruct, v) && 
        // distFromDest[u] != INT_MAX && 
        pCrawl->weight + distFromDest[u] < distFromDest[v])
      {
        printf("inside if\n");
        distFromDest[v] = distFromDest[u] + pCrawl->weight;
        pathSequence[v] = u;
        for(int i = 0; i < V; i++)
          printf("distFromDest[%d]: %d\n", i, distFromDest[i]);
        for(int i = 0; i < V; i++)
          printf("pathSequence[%d]: %d\n", i, pathSequence[i]);
        // update distance 
        // value in min heap also
        minHeap.DecreaseKey(minHeapStruct, v, distFromDest[v]);
      }
      pCrawl = pCrawl->next;

      printf("-------\n");
    }
    printf("##############\n");
  }
  minHeap.~MinHeap();

  printf("Final distFromDest: \n");
  for(int i = 0; i < V; i++)
    printf("distFromDest[%d]: %d\n", i, distFromDest[i]);

  printf("Final pathSequence: \n");
  for(int i = 0; i < V; i++)
    printf("pathSequence[%d]: %d\n", i, pathSequence[i]);

  int v = src;
  path.emplace_back(v);
  while(v != dest)
  {
    // path.insert(path.begin(), pathSequence[v]);
    path.emplace_back(pathSequence[v]);
    v = pathSequence[v];
  }
  // path.emplace_back(dest);
  // // print the calculated shortest distances
  // // PrintArr(dist, V);
  // printf("Path from %d to %d not found in the graph\n", src, dest);

  return path;
}





ModelGraph::~ModelGraph()
{
  for(int v = 0; v < V_; v++)
  {
    struct AdjList array = graph_->array[v];
    // printf("parent: %d\n", v);
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
  // printf("Cleanded ModelGraph Memory\n");
}
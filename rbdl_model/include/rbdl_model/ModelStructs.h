#ifndef MODELSTRUCTS_H
#define MODELSTRUCTS_H
  
// A structure to represent a 
// node in adjacency list
struct AdjListNode
{
    int dest;
    int weight;
    struct AdjListNode* next;
};
  
// A structure to represent 
// an adjacency list
struct AdjList
{
      
   // Pointer to head node of list
   struct AdjListNode *head; 
};
  
// A structure to represent a graph. 
// A graph is an array of adjacency lists.
// Size of array will be V (number of 
// vertices in graph)
struct Graph
{
    int V;
    struct AdjList* array;
};
#endif

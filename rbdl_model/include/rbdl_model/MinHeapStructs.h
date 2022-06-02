#ifndef MINHEAPSTRUCTS
#define MINHEAPSTRUCTS
// Structure to represent a min heap node
struct MinHeapNode
{
  int  v;
  int dist;
};
  
// Structure to represent a min heap
struct MinHeapStruct
{      
  // Number of heap nodes present currently
  int size;
  
  // Capacity of min heap
  int capacity;
  
  // This is needed for decreaseKey()
  int *pos;
  struct MinHeapNode **array;
};

#endif
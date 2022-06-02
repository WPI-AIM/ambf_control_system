#include "rbdl_model/MinHeap.h"


// A utility function to create a 
// new Min Heap Node
struct MinHeapNode* MinHeap::NewMinHeapNode(int v, int dist)
{
  struct MinHeapNode* minHeapNode =
    (struct MinHeapNode*) malloc(sizeof(struct MinHeapNode));
  minHeapNode->v = v;
  minHeapNode->dist = dist;
  return minHeapNode;
}
  
// A utility function to create a Min Heap
struct MinHeapStruct* MinHeap::CreateMinHeap(int capacity)
{
  struct MinHeapStruct* minHeapStruct =
    (struct MinHeapStruct*) malloc(sizeof(struct MinHeapStruct));
  minHeapStruct->pos = (int *)malloc(capacity * sizeof(int));
  minHeapStruct->size = 0;
  minHeapStruct->capacity = capacity;
  minHeapStruct->array =
    (struct MinHeapNode**) malloc(capacity * sizeof(struct MinHeapNode*));
  return minHeapStruct;
}
  
// A utility function to swap two 
// nodes of min heap. 
// Needed for min heapify
void MinHeap::SwapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b)
{
  struct MinHeapNode* t = *a;
  *a = *b;
  *b = t;
}
  
// A standard function to 
// heapify at given idx
// This function also updates 
// position of nodes when they are swapped.
// Position is needed for decreaseKey()
void MinHeap::MinHeapify(struct MinHeapStruct* minHeapStruct, int idx)
{
  int smallest, left, right;
  smallest = idx;
  left = 2 * idx + 1;
  right = 2 * idx + 2;

  if (left < minHeapStruct->size &&
      minHeapStruct->array[left]->dist < 
      minHeapStruct->array[smallest]->dist )
    smallest = left;

  if (right < minHeapStruct->size &&
      minHeapStruct->array[right]->dist <
      minHeapStruct->array[smallest]->dist )
    smallest = right;

  if (smallest != idx)
  {
    // The nodes to be swapped in min heap
    MinHeapNode *smallestNode = minHeapStruct->array[smallest];
    MinHeapNode *idxNode = minHeapStruct->array[idx];

    // Swap positions
    minHeapStruct->pos[smallestNode->v] = idx;
    minHeapStruct->pos[idxNode->v] = smallest;

    // Swap nodes
    SwapMinHeapNode(&minHeapStruct->array[smallest], &minHeapStruct->array[idx]);

    MinHeapify(minHeapStruct, smallest);
  }
}
  
// A utility function to check if 
// the given minHeap is ampty or not
int MinHeap::IsEmpty(struct MinHeapStruct* minHeapStruct)
{
  return minHeapStruct->size == 0;
}
  
// Standard function to extract 
// minimum node from heap
struct MinHeapNode* MinHeap::ExtractMin(struct MinHeapStruct* minHeapStruct)
{
  if (IsEmpty(minHeapStruct))
    return NULL;

  // Store the root node
  struct MinHeapNode* root = minHeapStruct->array[0];

  // Replace root node with last node
  struct MinHeapNode* lastNode = minHeapStruct->array[minHeapStruct->size - 1];
  minHeapStruct->array[0] = lastNode;

  // Update position of last node
  minHeapStruct->pos[root->v] = minHeapStruct->size-1;
  minHeapStruct->pos[lastNode->v] = 0;

  // Reduce heap size and heapify root
  --minHeapStruct->size;
  MinHeapify(minHeapStruct, 0);

  return root;
}
  
// Function to decreasy dist value 
// of a given vertex v. This function
// uses pos[] of min heap to get the
// current index of node in min heap
void MinHeap::DecreaseKey(struct MinHeapStruct* minHeapStruct, int v, int dist)
{
  // Get the index of v in  heap array
  int i = minHeapStruct->pos[v];

  // Get the node and update its dist value
  minHeapStruct->array[i]->dist = dist;

  // Travel up while the complete 
  // tree is not hepified.
  // This is a O(Logn) loop
  while (i && minHeapStruct->array[i]->dist < 
    minHeapStruct->array[(i - 1) / 2]->dist)
  {
    // Swap this node with its parent
    minHeapStruct->pos[minHeapStruct->array[i]->v] = (i-1)/2;
    minHeapStruct->pos[minHeapStruct->array[(i-1)/2]->v] = i;
    SwapMinHeapNode(&minHeapStruct->array[i], &minHeapStruct->array[(i - 1) / 2]);

    // move to parent index
    i = (i - 1) / 2;
  }
}
  
// A utility function to check if a given vertex
// 'v' is in min heap or not
bool MinHeap::IsInMinHeap(struct MinHeapStruct *minHeapStruct, int v)
{
  if (minHeapStruct->pos[v] < minHeapStruct->size)
    return true;
  return false;
}

// MinHeap::~MinHeap()
// {

// }
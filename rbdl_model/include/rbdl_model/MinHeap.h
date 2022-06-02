#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "rbdl_model/MinHeapStructs.h"

class MinHeap
{
public:
  MinHeap() = default;
  virtual ~MinHeap() = default;

  struct MinHeapNode* NewMinHeapNode(int v, int dist);
  struct MinHeapStruct* CreateMinHeap(int capacity);
  void SwapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b);
  void MinHeapify(struct MinHeapStruct* minHeapStruct, int idx);
  int IsEmpty(struct MinHeapStruct* minHeapStruct);
  struct MinHeapNode* ExtractMin(struct MinHeapStruct* minHeapStruct);
  void DecreaseKey(struct MinHeapStruct* minHeapStruct, int v, int dist);
  bool IsInMinHeap(struct MinHeapStruct *minHeapStruct, int v);
};
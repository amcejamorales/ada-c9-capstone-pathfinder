/*
 * Graph data structure 
 * Credit: adapted from Stanford University's C++ Library
*/

#ifndef _graph_h 
#define _graph_h 

#include "vector.h"

struct Node; 
struct Arc; 

struct Graph {
  Vector<Node *> nodes; 
  Vector<Arc *> arcs;
};

struct Node {
  int order; 
  Vector<float> distancesToWalls;
  Vector<char> openings;
  float distanceTravelled; 
  bool visited; 
  Vector<Arc *> arcs; 
};

struct Arc {
  Node *start; 
  Node *finish; 
  char direction; 
  float distanceBetweenNodes;
};

#endif 














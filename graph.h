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

void addNode(Graph & g, int order, Vector<float> distancesToWalls, Vector<char> openings, float distanceTravelled) {
    Node *node = new Node;
    node->order = order;
    node->distancesToWalls = distancesToWalls;
    node->openings = openings;
    node->distanceTravelled = distanceTravelled;
    node->visited = true;
    g.nodes.add(node);
}

void addArc(Graph & g, Node *startNode, Node *finishNode, char direction, float distanceBetweenNodes) {
    Arc *arc = new Arc;
    arc->start = startNode;
    arc->finish = finishNode;
    arc->direction = direction;
    arc->distanceBetweenNodes = distanceBetweenNodes;
    g.arcs.add(arc);
    // do I need to add this?
      startNode->arcs.add(arc);
      finishNode->arcs.add(arc); // this would have to be a reversed arc
}

#endif 














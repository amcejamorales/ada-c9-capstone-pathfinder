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

void printGraphNodes(Graph & g) {
    Vector<Node *> graphNodes = g.nodes;
    Serial.println("***************************");
    Serial.print("Printing ");
    Serial.print(graphNodes.size());
    Serial.println(" nodes for newGraph.");
    
    for(int i = 0; i < graphNodes.size(); i++) {
        Node *currentNode = graphNodes.get(i);
        Vector<float> distancesToWalls = currentNode->distancesToWalls;
        Vector<char> currentOpenings = currentNode->openings;
        
        Serial.print("Node: ");
        Serial.println(currentNode->order);
        
        Serial.print("\tNode Distances to Walls: ");
        Serial.print("F: ");
        Serial.print(distancesToWalls[0]);
        Serial.print(", L: ");
        Serial.print(distancesToWalls[1]);
        Serial.print(", R: ");
        Serial.println(distancesToWalls[2]);
        
        Serial.print("\tNode openings: ");
        for(int j = 0; j < currentOpenings.size(); j++) {
            if (j > 0) { Serial.print(", "); }
            Serial.print(currentOpenings[j]);
        }
        Serial.println("");
        
        Serial.print("\tDistance Travelled: ");
        Serial.println(currentNode->distanceTravelled);
        
        Serial.print("\tVisited: ");
        Serial.println(currentNode->visited);
    }
    
    Serial.println("***************************");
    
}

void printGraphArcs(Graph & g) {
    Vector<Arc *> graphArcs = g.arcs;
    Serial.println("***************************");
    Serial.print("Printing ");
    Serial.print(graphArcs.size());
    Serial.println(" arcs for newGraph.");
    
    for(int i = 0; i < graphArcs.size(); i++) {
        Arc *currentArc = graphArcs.get(i);
        
        Serial.print("Arc ");
        Serial.println(i + 1);
        
        Serial.print("\tStarting Node: ");
        Serial.println(currentArc->start->order);
        
        Serial.print("\tEnding Node: ");
        Serial.println(currentArc->finish->order);
        
        Serial.print("\tDirection from start to finish: ");
        Serial.println(currentArc->direction);
        
        Serial.print("\tDistance between start and finish: ");
        Serial.println(currentArc->distanceBetweenNodes);
    }
    
    Serial.println("***************************");
}

#endif 














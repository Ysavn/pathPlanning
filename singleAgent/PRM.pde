//***************************************************README***************************************************
//Approach: A*
//One Hack: Treat goalPos as a node in graph & hence connecting all non-colliding neighboring node paths to it
//Thus one additional change in Line 197 required for parent array i.e. (its size => maxNumNodes + 1)
//************************************************************************************************************

import java.util.PriorityQueue; 
import java.util.Map; 

float[] costSoFar = new float[maxNumNodes+1];
float[] hCost = new float[maxNumNodes+1];
float[][] W = new float[maxNumNodes+1][maxNumNodes+1];
PriorityQueue<Obj> Q = new PriorityQueue<Obj>();
ArrayList<Integer> allStartIDs = new ArrayList<Integer>();
//ArrayList<Integer> allGoalIDs = new ArrayList<Integer>();

public class Obj implements Comparable<Obj>{
  private Float key; //cost
  private Integer val; //index of node
  
  public Obj(Float k, Integer v){
    key = k;
    val = v;
  }
  
  public Float getKey(){
    return key;
  }
  
  public Integer getVal(){
    return val;
  }
  
  @Override
  public int compareTo(Obj o2){
    return this.getKey().compareTo(o2.getKey());   
  }
}

ArrayList<Integer> runAStar(ArrayList<Integer> startIDs, int numNodes, Vec2 startPos, int goalID){
  
  Q.clear();
  
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i <= numNodes; i++) { 
    parent[i] = -2; //No parent yet
    costSoFar[i] = 999999;
  }
  
  for(int i=0;i<startIDs.size();i++){
    
    int startID = startIDs.get(i);
    Vec2 stNode = nodePos[startID];
    float dist = stNode.distanceTo(startPos);
    costSoFar[startID] = dist;
    float priority = dist + W[startID][numNodes];
    Q.add(new Obj(priority, startID));
    parent[startID] = -1;
  }
  
  while(Q.size() > 0){
    
    Obj curr = Q.poll();
    int curr_idx = curr.getVal();
    
    if(curr_idx == goalID) break;
    
    for(int i=0;i<neighbors[curr_idx].size(); i++){
      int next_idx = neighbors[curr_idx].get(i);
      float new_cost = costSoFar[curr_idx] + W[curr_idx][next_idx];
      if(new_cost < costSoFar[next_idx]){
        
        boolean flag = (costSoFar[next_idx] == 999999);
        float old_p = costSoFar[next_idx] + W[next_idx][numNodes];
        float new_p = new_cost + W[next_idx][numNodes];
        costSoFar[next_idx] = new_cost;
        if(!flag) Q.remove(new Obj(old_p, next_idx));
        Q.add(new Obj(new_p, next_idx));
        parent[next_idx] = curr_idx;
      }
    }
  }
  
  int prevNode = parent[goalID];
  if(prevNode == -2){
    //No path
    path.add(0, -1);
    return path;
  }
  
  while (prevNode >= 0){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  return path;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  allStartIDs.clear();
  
  Vec2 dir = goalPos.minus(startPos).normalized();
  float distBetween = goalPos.distanceTo(startPos);
  hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startPos, dir, distBetween);
  //no collisons in direct path from startPos to goalPos
  if(!circleListCheck.hit) return path;
  
  //all prospective start IDs
  for(int i=0;i<numNodes;i++)
  {
    Vec2 stNode = nodePos[i];
    dir = stNode.minus(startPos).normalized();
    distBetween = stNode.distanceTo(startPos);
    circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startPos, dir, distBetween);
    if(!circleListCheck.hit){
      allStartIDs.add(i);
    }
  }

  Map<Integer, Integer> mapID = new HashMap<Integer, Integer>();
  for(int i=0;i<numNodes;i++)
  {
    Vec2 endNode = nodePos[i];
    dir = goalPos.minus(endNode).normalized();
    distBetween = goalPos.distanceTo(endNode);
    circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, endNode, dir, distBetween);
    if(!circleListCheck.hit){
      neighbors[i].add(numNodes);
      mapID.put(i, neighbors[i].size()-1);
    }
  }
  
  for(int i=0;i<numNodes;i++){
    for(int j=0;j<numNodes;j++){
      W[i][j] = nodePos[i].distanceTo(nodePos[j]);
    }
    W[i][numNodes] = nodePos[i].distanceTo(goalPos);
  }
  W[numNodes][numNodes] = 0;
  
  path = runAStar(allStartIDs, numNodes, startPos, numNodes);
  
  for (Integer id : mapID.keySet()) {
    int pos = mapID.get(id);
    neighbors[id].remove(pos);
  }
 
  return path;
}

//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes+1]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}

/*
ArrayList<Integer> runAStar(int numNodes, int startID, int goalID){
  
  Q.clear();
  
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { 
    visited[i] = false;
    parent[i] = -1; //No parent yet
    costSoFar[i] = 999999;
    hCost[i] = W[i][goalID];
  }
  
  costSoFar[startID] = 0;
  
  Q.add(new Obj(0.0, startID));
  while(Q.size() > 0){
    Obj curr = Q.poll();
    int curr_idx = curr.getVal();
    if(curr_idx == goalID) break;
    for(int i=0;i<neighbors[curr_idx].size(); i++){
      int next_idx = neighbors[curr_idx].get(i);
      float new_cost = costSoFar[curr_idx] + W[curr_idx][next_idx];
      if(new_cost < costSoFar[next_idx]){
        
        boolean flag = (costSoFar[next_idx] == 999999);
        float old_p = costSoFar[next_idx] + hCost[next_idx];
        float new_p = new_cost + hCost[next_idx];
        costSoFar[next_idx] = new_cost;
        if(!flag) Q.remove(new Obj(old_p, next_idx));
        Q.add(new Obj(new_p, next_idx));
        parent[next_idx] = curr_idx;
      }
    }
  }
  
  int prevNode = parent[goalID];
  if(prevNode == -1 && goalID!=startID){
    //No path
    path.add(0, -1);
    return path;
  }
  path.add(0,goalID);
  while (prevNode >= 0){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  return path;
}

//referred to pathQuality in HW3_Test
float getPathLength(ArrayList<Integer> curPath, Vec2 startPos, Vec2 goalPos){
 
  float segmentLength = 99999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return segmentLength; //No path found  
  
  float pathLength = 0;
  
  segmentLength = startPos.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  
  
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;
  }
  
  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(goalPos);
  pathLength += segmentLength;
  
  return pathLength;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  allStartIDs.clear();
  allGoalIDs.clear();
  
  Vec2 dir = goalPos.minus(startPos).normalized();
  float distBetween = goalPos.distanceTo(startPos);
  hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startPos, dir, distBetween);
  //no collisons in direct path from startPos to goalPos
  if(!circleListCheck.hit) return path;
  
  //all prospective start IDs
  int threshold = 1000;

  for(int i=0;i<numNodes;i++)
  {
    Vec2 stNode = nodePos[i];
    dir = stNode.minus(startPos).normalized();
    distBetween = stNode.distanceTo(startPos);
    circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startPos, dir, distBetween);
    if(!circleListCheck.hit && distBetween < threshold){
      allStartIDs.add(i);
    }
  }

  //all prospective goal IDs
  for(int i=0;i<numNodes;i++)
  {
    Vec2 endNode = nodePos[i];
    dir = goalPos.minus(endNode).normalized();
    distBetween = goalPos.distanceTo(endNode);
    circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, endNode, dir, distBetween);
    if(!circleListCheck.hit && distBetween < threshold){
      allGoalIDs.add(i);
    }
  }

  for(int i=0;i<numNodes;i++){
    for(int j=0;j<numNodes;j++){
      W[i][j] = nodePos[i].distanceTo(nodePos[j]);
    }
  }
  
  int startID, goalID;
  float pathLen = 99999;
  ArrayList<Integer> shortestPath = new ArrayList<Integer>();
  for(int i=0;i<allStartIDs.size();i++){
    for(int j=0;j<allGoalIDs.size();j++){
      startID = allStartIDs.get(i);
      goalID = allGoalIDs.get(j);
      path = runAStar(numNodes, startID, goalID);
      float tmpLen = getPathLength(path, startPos, goalPos);
      if(tmpLen < pathLen){
        pathLen = tmpLen;
        shortestPath = path;
      }
    }
  }
  
  if(shortestPath.size()==0){
    shortestPath.add(-1);
  }
  return shortestPath;
}

ArrayList<Integer> runUCS(int numNodes, int startID, int goalID){
  
  Q.clear();
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { 
    visited[i] = false;
    parent[i] = -1; //No parent yet
    costSoFar[i] = 999999;
  }
 
  costSoFar[startID] = 0;
  
  Q.add(new Obj(0.0, startID));
  while(Q.size() > 0){
    Obj curr = Q.poll();
    int curr_idx = curr.getVal();
    if(curr_idx == goalID) break;
    for(int i=0;i<neighbors[curr_idx].size(); i++){
      int next_idx = neighbors[curr_idx].get(i);
      float new_cost = costSoFar[curr_idx] + W[curr_idx][next_idx];
      if(new_cost < costSoFar[next_idx]){
        
        boolean flag = costSoFar[next_idx] == 999999;
        float old_p = costSoFar[next_idx];
        float new_p = new_cost;
        costSoFar[next_idx] = new_cost;
        if(!flag) Q.remove(new Obj(old_p, next_idx));
        Q.add(new Obj(new_p, next_idx));
        parent[next_idx] = curr_idx;
      }
    }
  }
  
  int prevNode = parent[goalID];
  if(prevNode == -1 && goalID!=startID){
    //No path
    path.add(0, -1);
    return path;
  }
  path.add(0,goalID);
  while (prevNode >= 0){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  return path;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  
  path = runBFS(nodePos, numNodes, startID, goalID);
  
  return path;
}
*/

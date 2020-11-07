int numObstacles = 20;
int numNodes  = 100;
float ra = 20;
float speed = 40;
float dt = 0.05;
int idx = 0;
int a = 145;
int maxNumObstacles = 1000;
int maxNumNodes = 1000;

ArrayList<Vec2> path = new ArrayList<Vec2> ();
ArrayList<Integer> curPath;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; 
float circleRad[] = new float[maxNumObstacles]; 
Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);
Vec2[] nodePos = new Vec2[maxNumNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
   
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeOrderedObstacles(int numObstaclesInRow){
  
  int row = numObstaclesInRow, col = numObstaclesInRow + 1, w = 100, r = 25;
  int idx = 0;
  for(int i=0;i<col;i++){
    int x = w*(i+1) + r*(2*i+1);
    int y = ((i%2==0)?w:w/2) + r; 
    for(int j=0;j<row;j++){
      circlePos[idx] = new Vec2(x, y);
      circleRad[idx] = r;
      y += w + 2*r;
      idx++;
    }
  }
}

void addObstacleAtPosition(float x, float y){
  circlePos[numObstacles] = new Vec2(x, y);
  circleRad[numObstacles] = 25;
  numObstacles++;
}

void updateFirstObstaclePosition(float x, float y){
  circlePos[0] = new Vec2(x, y);
}


int strokeWidth = 2;
void setup(){
  size(850,700, P3D);
  camera = new Camera();
  img = loadImage("neon_slime.png");
  img2 = loadImage("basketball.jpg");
  float angle = 360.0 / tubeRes;
  for (int i = 0; i <= tubeRes; i++) {
    tubeX[i] = cos(radians(i * angle));
    tubeY[i] = sin(radians(i * angle));
  }
  noStroke();
  setupPRM();
}

int numCollisions;
float pathLength;
boolean reachedGoal;

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  }
  return randPos;
}

ArrayList<Vec2> getPathList(ArrayList<Integer> pathLoc){
  ArrayList<Vec2> path = new ArrayList<Vec2>();
  for(int i=0;i<pathLoc.size();i++){
    int id = pathLoc.get(i);
    if(id==-1)
      continue;
    path.add(nodePos[id]);
  }
  path.add(goalPos);
  return path;
}

void moveAlongPath(float dt){
  
  Vec2 vel = new Vec2(0, 0);
  float dist = 0;
  hitInfo hitCheck = new hitInfo();
  hitCheck.hit = true;
  
  if(idx == path.size()){
    startPos = goalPos;
    return;
  }
  if(idx < path.size()-1){
    //shortcut
    Vec2 dir = path.get(idx+1).minus(startPos).normalized();
    dist = path.get(idx+1).minus(startPos).length();
    hitCheck = rayCircleListIntersect(circlePos, circleRad,  numObstacles, startPos, dir, dist);
    if(!hitCheck.hit){
      vel = dir.times(speed);
      idx++;
    }
  }
  
  if(hitCheck.hit){
    Vec2 dir = path.get(idx).minus(startPos).normalized();
    dist = path.get(idx).minus(startPos).length();
    vel = dir.times(speed);
  }
  if(dist < 2){
    vel = new Vec2(0, 0);
    idx++;
  }
  startPos.add(vel.times(dt));
}

void setupPRM(){

  placeOrderedObstacles(4);
  
  startPos = sampleFreePos();
  goalPos = startPos;

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  path = getPathList(curPath);
}

void draw(){

  specular(120, 120, 180);  //Setup lights… 
  ambientLight(190,190,190);   //More light…
  lightSpecular(255,255,255); shininess(10);  //More light…
  directionalLight(100, 100, 100, 0, 0, -1); //More light…
  
  for(int i=0;i<2;i++)
    camera.Update(1.0/frameRate);
  strokeWeight(1);
  
  background(200); //Grey background
  ptsW=20;
  ptsH=20;
  sphereAngY = PI;
  sphereAngDeltaY = 0;
  initializeSphere(ptsW, ptsH);
  fill(255,255,255);
  
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    drawCylinder(c.x, c.y, r);
  }
  
  //Draw Start and Goal
  fill(20,60,250);
  drawSphere(startPos.x, startPos.y);
  fill(250,30,50);
  circle(goalPos.x,goalPos.y,ra*2);
  if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found
  
  moveAlongPath(dt);
}

void keyPressed(){
  camera.HandleKeyPressed();
}

void mouseClicked(){
  if (mouseButton == RIGHT){
    addObstacleAtPosition(mouseX, mouseY);
    generateRandomNodes(numNodes, circlePos, circleRad);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  }
  else{
    goalPos = new Vec2(mouseX, mouseY);
  }
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  path = getPathList(curPath);
  idx = 0;
}

void mouseDragged() 
{
  updateFirstObstaclePosition(mouseX, mouseY);
  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  path = getPathList(curPath);
  idx = 0;
}

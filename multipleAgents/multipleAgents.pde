PImage img3;
int numObstacles = 20;
int numNodes  = 100;
float ra = 20;
float speed = 40;
float dt = 0.05;
ArrayList<Vec2> path1 = new ArrayList<Vec2> ();
ArrayList<Vec2> path2 = new ArrayList<Vec2> ();
int idx1 = 0;
int idx2 = 0;
int numCollisions;
float pathLength;
boolean reachedGoal;
  
int a = 145;
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; 
float circleRad[] = new float[maxNumObstacles];

Vec2 startPos = new Vec2(-1,-1);
Vec2 goalPos = new Vec2(-1,-1);

Vec2 startPos2 = new Vec2(-1,-1);
Vec2 goalPos2 = new Vec2(-1,-1);

boolean moveIt = false;

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

boolean insideAgent(Vec2 pos, Vec2 agentPos1, Vec2 agentPos2){
  float r = 2*ra;
  float dist1 = pos.distanceTo(agentPos1);
  float dist2 = pos.distanceTo(agentPos2);
  if(dist1 >= r+2 && dist2 >= r+2) return false;
  return true;
}

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii, Vec2 agentPos1, Vec2 agentPos2){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos) || insideAgent(randPos, agentPos1, agentPos2);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos) || insideAgent(randPos, agentPos1, agentPos2);
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

ArrayList<Integer> curPath1;
ArrayList<Integer> curPath2;

int strokeWidth = 2;
void setup(){
  size(850,700, P3D);
  img = loadImage("neon_slime.png");
  img2 = loadImage("basketball1.jpg");
  img3 = loadImage("basketball2.jpg");
  float angle = 360.0 / tubeRes;
  for (int i = 0; i <= tubeRes; i++) {
    tubeX[i] = cos(radians(i * angle));
    tubeY[i] = sin(radians(i * angle));
  }
  noStroke();
  placeOrderedObstacles(4);
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  }
  return randPos;
}

ArrayList<Vec2> getPathList(ArrayList<Integer> pathLoc, Vec2 goalPos){
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

void moveAlongPath1(float dt){
  
  Vec2 vel = new Vec2(0, 0);
  float dist = 0;
  boolean hitCheck = true;
  
  if(idx1 == path1.size()){
    startPos = goalPos;
    return;
  }
  if(idx1 < path1.size()-1){
    //shortcut
    Vec2 dir = path1.get(idx1+1).minus(startPos).normalized();
    dist = path1.get(idx1+1).minus(startPos).length();
    hitCheck = rayCircleListIntersect(circlePos, circleRad,  numObstacles, startPos, dir, dist).hit || rayCircleIntersect(startPos2, ra, startPos, dir, dist).hit;
    if(!hitCheck){
      vel = dir.times(speed);
      idx1++;
    }
  }
  
  if(hitCheck){
    Vec2 dir = path1.get(idx1).minus(startPos).normalized();
    dist = path1.get(idx1).minus(startPos).length();
    vel = dir.times(speed);
  }
  if(dist < 2){
    vel = new Vec2(0, 0);
    idx1++;
  }
  startPos.add(vel.times(dt));
}

void moveAlongPath2(float dt){
  
  Vec2 vel = new Vec2(0, 0);
  float dist = 0;
  boolean hitCheck = true;
  
  if(idx2 == path2.size()){
    startPos2 = goalPos2;
    return;
  }
  if(idx2 < path2.size()-1){
    //shortcut
    Vec2 dir = path2.get(idx2+1).minus(startPos2).normalized();
    dist = path2.get(idx2+1).minus(startPos2).length();
    hitCheck = rayCircleListIntersect(circlePos, circleRad,  numObstacles, startPos2, dir, dist).hit || rayCircleIntersect(startPos, ra, startPos2, dir, dist).hit;
    if(!hitCheck){
      vel = dir.times(speed);
      idx2++;
    }
  }
  
  if(hitCheck){
    Vec2 dir = path2.get(idx2).minus(startPos2).normalized();
    dist = path2.get(idx2).minus(startPos2).length();
    vel = dir.times(speed);
  }
  if(dist < 2){
    vel = new Vec2(0, 0);
    idx2++;
  }
  startPos2.add(vel.times(dt));
}

void testPRM(){
  generateRandomNodes(numNodes, circlePos, circleRad, startPos, startPos2);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
}

boolean withinRange(Vec2 pos1, Vec2 pos2){
  float dist = pos1.distanceTo(pos2);
  if(dist < 60) {
    return true;
  }
  return false;
}

int cnt = 0;
void draw(){

  specular(120, 120, 180);  //Setup lights… 
  ambientLight(190,190,190);   //More light…
  lightSpecular(255,255,255); shininess(10);  //More light…
  directionalLight(100, 100, 100, 0, 0, -1); //More light…
  
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
  if(startPos.x!=-1 && startPos.y!=-1) drawSphere(startPos.x, startPos.y, img2);
  if(startPos2.x!=-1 && startPos2.y!=-1) drawSphere(startPos2.x, startPos2.y, img3);
  fill(250,128,50);
  if(goalPos.x!=-1 && goalPos.y!=-1) circle(goalPos.x,goalPos.y,ra*2);
  fill(127, 0, 255);
  if(goalPos2.x!=-1 && goalPos2.y!=-1) circle(goalPos2.x,goalPos2.y,ra*2);
  
  if(moveIt){
    
    if(cnt==0 && withinRange(startPos, startPos2)){
      println("yes");
      testPRM();
      if(startPos.distanceTo(goalPos)!=0){
        curPath1 = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes, startPos2);
        path1 = getPathList(curPath1, goalPos);
        idx1 = 0;
        moveAlongPath2(dt);
      }
      else{
        curPath2 = planPath(startPos2, goalPos2, circlePos, circleRad, numObstacles, nodePos, numNodes, startPos);
        path2 = getPathList(curPath2, goalPos2);
        idx2 = 0;
        moveAlongPath1(dt);
      }
      cnt++;
    }
    else{
      moveAlongPath1(dt);
      moveAlongPath2(dt);
    }
    if(cnt>0)
      cnt++;
    cnt = cnt % 15;
  }
}

boolean shiftDown = false;
boolean onePressed, twoPressed, threePressed, fourPressed;
void keyPressed(){
 
  onePressed = twoPressed = threePressed = fourPressed = false;
  moveIt = false;
  if(key=='1'){
    onePressed = true;
  }
  if(key=='2'){
    twoPressed = true;
  }
  if(key=='3'){
    threePressed = true;
  }
  if(key=='4'){
    fourPressed = true;
  }
  if(key=='g'){
    testPRM();
    curPath1 = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes, startPos2);
    path1 = getPathList(curPath1, goalPos);
    curPath2 = planPath(startPos2, goalPos2, circlePos, circleRad, numObstacles, nodePos, numNodes, startPos);
    path2 = getPathList(curPath2, goalPos2);
    idx1 = 0;
    idx2 = 0;
    moveIt = true;
  }
  
}

void mouseClicked(){
  if(onePressed) startPos = new Vec2(mouseX, mouseY);
  if(twoPressed) startPos2 = new Vec2(mouseX, mouseY);
  if(threePressed) goalPos = new Vec2(mouseX, mouseY);
  if(fourPressed) goalPos2 = new Vec2(mouseX, mouseY);
  onePressed = twoPressed = threePressed = fourPressed = false;
}

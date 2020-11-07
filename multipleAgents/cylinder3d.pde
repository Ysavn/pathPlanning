int tubeRes = 32;
float[] tubeX = new float[tubeRes+1];
float[] tubeY = new float[tubeRes+1];
PImage img;

void drawCylinder(float xpos, float ypos, float r) {
  //background(0);
  pushMatrix();
  noStroke();
  noFill();
  translate(xpos, ypos);
  //rotateX(map(mouseY, 0, height, -PI, PI));
  //rotateY(map(mouseX, 0, width, -PI, PI));
  
  beginShape(QUAD_STRIP);
  texture(img);
  for (int i = 0; i <= tubeRes; i++) {
    float x = tubeX[i] * r;
    float y = tubeY[i] * r;
    float u = img.width / tubeRes * i;
    vertex(x, y, 0, u, 0);
    vertex(x, y, 50, u, img.height);
  }
  endShape();
  /*(
  beginShape();
  texture(img);
  for (int i = 0; i <= tubeRes; i++) {
    vertex(tubeX[i]*r, tubeY[i]*r, -r);
  }
  endShape(CLOSE);
  */
  beginShape();
  texture(img);
  for (int i = 0; i <= tubeRes; i++) {
    vertex(tubeX[i]*r, tubeY[i]*r, 50);
  }
  endShape(CLOSE);
  popMatrix();
}

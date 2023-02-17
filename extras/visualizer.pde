import processing.serial.*;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float deltax = 0.0, deltay = 0.0;
float sx = 0.0, sy = 0.0, sz = 0.0;

int scaling_factor = 100;

void setup()
{
  size(1000, 1000, P3D);

  myPort = new Serial(this, "/dev/ttyACM0", 115200);
  myPort.bufferUntil('\n');

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void draw()
{
  background(255); // set background to white
  lights();
  
  deltax += sx * -scaling_factor;
  deltay += sz * scaling_factor;
  float deltax_positive = deltax > 0 ? 0 : 1;
  float deltay_positive = deltay > 0 ? 0 : 1;
  if (abs(deltax) >=  250.0) {
    deltax = 250 * pow(-1, deltax_positive);
  }
  if (abs(deltay) >= 250.0) {
    deltay = 250 * pow(-1, deltay_positive);
  }

  translate(width/2 + deltax, height/2 + deltay); // set position to centre

  pushMatrix(); // begin object

  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawArduino();

  popMatrix(); // end of object
}

void serialEvent(Serial myPort) {

  String myString = myPort.readStringUntil('\n');
  myString = trim(myString);
  float sensors[] = float(split(myString, ':'));
  
  pitch = sensors[0];
  roll = sensors[1];
  yaw = sensors[2];
  sx = sensors[3];
  sy = sensors[4];
  sz = sensors[5];

}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape

  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
}

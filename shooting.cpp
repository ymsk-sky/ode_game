#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <time.h>
#include <stdlib.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;
dsFunctions fn;

int fire = 0;
double start_x = 0.0, start_y = 0.5, start_z = 0.5;
double R[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};
double target[3] = {0.0, 0.0, 0.5};
double robot[3] = {start_x, start_y, start_z};
double bullet[3] = {start_x, start_y, start_z};
double sides[3] = {1.0, 0.5, 1.0};

void start()
{
  static float xyz[3] = {-0.1, 0.0, 20.0};
  static float hpr[3] = {-180, -90, -90};

  dsSetViewpoint(xyz, hpr);
  dsSetSphereQuality(3);
  dRSetIdentity(R);
}

static void command(int cmd)
{
  switch(cmd) {
    case 'a':
      robot[0] -= 1.0;
      if(fire == 0) bullet[0] -= 1.0;
      break;
    case 'd':
      robot[0] += 1.0;
      if(fire == 0) bullet[0] += 1.0;
      break;
    case 'x':
      robot[1] -= 1.0;
      break;
    case 'w':
      robot[1] += 1.0;
      break;
    case 32:
      fire = 1;
      break;
    default:
      printf("Input %c (%d)\n", (char)cmd, cmd);
  }
}

void setDrawStuff(void (*function)(int pause), void (*function2)(int cmd))
{
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = function;
  fn.command = &command;
  fn.path_to_textures = "../textures";
}

void drawTarget()
{
  int i, j, num = 10;
  for(i=0; i<num; i++) {
    dsSetColor(1.0, 0.0, 0.0);
    target[0] = 2 * i + start_x - num + 1;
    target[1] = start_y + 10;
    target[2] = 0.5;
    dsDrawBox(target, R, sides);
  }
}

void drawBullet()
{
  dsSetColor(0.0, 0.0, 1.0);
  dsDrawSphere(bullet, R, 0.1);
}

void dmLoop(int w, int h, void (*function)(int pause), void (*function2)(int cmd))
{
  setDrawStuff(function, command);
  dsSimulationLoop(0, 0, w, h, &fn);
}

void simLoop(int pause)
{
  dsSetColor(1.0, 1.0, 0.0);
  dsDrawSphere(robot, R, 0.5);

  if(fire) bullet[1] += 0.01;
  drawBullet();
  drawTarget();
}

int main()
{
  robot[1] = bullet[1] = start_y -10;
  dmLoop(800, 600, simLoop, command);
  return 0;
}

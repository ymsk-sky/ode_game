#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>
#include "baseblock.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#endif

#define BLOCK_SIZE 0.2
#define BLOCK_MASS 0.1

dWorldID world;
dSpaceID space;
dGeomID ground, frontwall, backwall;
dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {
  dBodyID body;
  dGeomID geom;
  int rotate;
  int shape[4][4];
} BLOCK;

BLOCK wall[22][12];
BLOCK block[4][4];

dJointID wall_joint[22][12];
dJointID block_joint[4];

dReal block_l[3] = {BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE};

double fall_speed = -5.0;
bool fall_flag = false;

//draw
void drawBlock()
{

}

void drawWall()
{
  dsSetColor(0.1, 0.1, 0.1);
  for(int i=0; i<22; i++) {
    for(int j=0; j<12; j++) {
      if(stage[i][j] == 1) {
        dsDrawBox(dBodyGetPosition(wall[i][j].body),
                  dBodyGetRotation(wall[i][j].body), block_l);
      }
    }
  }
}

//create
void createBlock()
{

}

void createWall()
{
  dMass mass;
  for(int i=0; i<22; i++) {
    for(int j=0; j<12; j++) {
      if(stage[i][j] == 1) {
        wall[i][j].body = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, BLOCK_MASS, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
        dBodySetMass(wall[i][j].body, &mass);
        dBodySetPosition(wall[i][j].body, -BLOCK_SIZE/2,
          (j-6)*BLOCK_SIZE - BLOCK_SIZE/2, (22-i)*BLOCK_SIZE - BLOCK_SIZE/2);
        wall[i][j].geom = dCreateBox(space, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);
        dGeomSetBody(wall[i][j].geom, wall[i][j].body);

        wall_joint[i][j] = dJointCreateFixed(world, 0);
        dJointAttach(wall_joint[i][j], wall[i][j].body, 0);
        dJointSetFixed(wall_joint[i][j]);
      }
    }
  }
}

static void command(int cmd)
{
  switch(cmd) {
    case ' ':
      break;
  }
}

static void start()
{
  float xyz[3] = {10.0, 0.0, 2.0};
  float hpr[3] = {-180.0, 0.0, 0.0};

  dsSetViewpoint(xyz, hpr);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10;
  dContact contact[N];

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  for(int i=0; i<n; i++) {
    contact[i].surface.mu = dInfinity;            // 摩擦係数
    contact[i].surface.mode = dContactBounce;     // 接触面の反発性を設定
    contact[i].surface.bounce = 0.2;              // 反発係数
    contact[i].surface.bounce_vel = 0.0;          // 反発最低速度
    //contact[i].surface.soft_erp = 0.2;            // 関節誤差修正
    //contact[i].surface.soft_cfm = 1e-5;           // 拘束力混合

    dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
    dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                    dGeomGetBody(contact[i].geom.g2));
  }
}

static void simLoop(int pause)
{
  if(!pause) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, 0.01);
    dJointGroupEmpty(contactgroup);
  }
  drawWall();
  if(fall_flag) drawBlock();
}

static void setDrawStuff()
{
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.path_to_textures = "../textures";
}

int main(int argc, char *argv[])
{
  setDrawStuff();
  dInitODE();
  world = dWorldCreate();
  dWorldSetGravity(world, 0.0, 0.0, fall_speed);

  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  ground = dCreatePlane(space, 0, 0, 1, 0);
  frontwall = dCreatePlane(space, 1, 0, 0, 0);
  backwall = dCreatePlane(space, 1, 0, 0, -BLOCK_SIZE);

  createWall();

  dsSimulationLoop(argc, argv, 400, 800, &fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}

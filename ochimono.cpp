#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>
#include <assert.h>
#include "blocks.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#endif

dWorldID world;
dSpaceID space;
dGeomID ground;
dJointGroupID contactgroup;
dsFunctions fn;

typedef struct {
  dBodyID body;
  dGeomID geom;
  int rotate;
  int shape[4][4];
} BLOCK;

double fall_speed = 0.0;

BLOCK wall[22][12];
BLOCK block[4][4];

dJointID wall_j[22][12];

dReal block_l[3] = {1, 1, 1};

void drawBlock()
{
  dsSetColor(1.0, 0.0, 0.0);
  for(int i=0; i<4; i++) {
    for(int j=0; j<4; j++) {
      if(blocks[0][i][j] == 1) {
        dsDrawBox(dBodyGetPosition(block[i][j].body),
        dBodyGetRotation(block[i][j].body), block_l);
      }
    }
  }
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

void createStage()
{
  dMass mass;

  for(int i=0; i<22; i++) {
    for(int j=0; j<12; j++) {
      if(stage[i][j] == 1) {
        wall[i][j].body = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, 1, block_l[0], block_l[1], block_l[2]);
        dBodySetMass(wall[i][j].body, &mass);
        dBodySetPosition(wall[i][j].body, i, j, 0.5);
        wall[i][j].geom = dCreateBox(space, block_l[0], block_l[1], block_l[2]);
        dGeomSetBody(wall[i][j].geom, wall[i][j].body);

        wall_j[i][j] = dJointCreateFixed(world, 0);
        dJointAttach(wall_j[i][j], wall[i][j].body, 0);
        dJointSetFixed(wall_j[i][j]);
      }
    }
  }
}

void createBlock(int n)
{
  dMass mass;

  for(int i=0; i<4; i++) {
    for(int j=0; j<4; j++) {
      if(blocks[n][i][j] == 1) {
        block[i][j].body = dBodyCreate(world);
        dMassSetZero(&mass);
        dMassSetBoxTotal(&mass, 1, block_l[0], block_l[1], block_l[2]);
        dBodySetMass(block[i][j].body, &mass);
        dBodySetPosition(block[i][j].body, i+5, j+5, 0.5);

        block[i][j].geom = dCreateBox(space, block_l[0], block_l[1], block_l[2]);
        dGeomSetBody(block[i][j].geom, block[i][j].body);
      }
    }
  }
}

static void command(int cmd)
{
  switch(cmd) {
    case 'w': // ↑
      break;
    case 'a': // ←
      break;
    case 's': // ↓
      break;
    case 'd': // →
      break;
    case ' ': // space
      printf("space!\n");
      break;
    case 'z': // speed_up
      dWorldSetGravity(world, ++fall_speed, 0, 0);
      break;
    default:
      break;
  }
}

static void start()
{
  float xyz[3] = {10.0, 5.5, 20.0};
  float hpr[3] = {0.0, -90.0, 180.0};

  dsSetViewpoint(xyz, hpr);
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int N = 10;  // 同時に衝突する可能性のある点数
  dContact contact[N];

  // 接触している物体のどちらかが地面ならisGroundに非0をセット
  int isGround = ((ground == o1) || (ground == o2));

  // 2つの剛体がジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  // 衝突情報の生成
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  for(int i=0; i<n; i++) {
    // TODO 跳ね返り挙動がおかしいのでパラメータ調整

    contact[i].surface.mu = dInfinity;            // 摩擦係数
    contact[i].surface.mode = dContactBounce;     // 接触面の反発性を設定
    contact[i].surface.bounce = 0.0;              // 反発係数
    contact[i].surface.bounce_vel = 0.0;          // 反発最低速度
    //contact[i].surface.soft_erp = 0.2;            // 関節誤差修正
    //contact[i].surface.soft_cfm = 1e-5;           // 拘束力混合

    // 接触ジョイントの生成
    dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
    // 接触している2つの剛体を接触ジョイントにより拘束
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
  drawBlock();
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
  dWorldSetGravity(world, fall_speed, 0, 0);

  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  ground = dCreatePlane(space, 0, 0, 1, 0);

  createStage();
  createBlock(0);

  dsSimulationLoop(argc, argv, 400, 800, &fn);
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}

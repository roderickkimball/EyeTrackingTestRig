#include "KinematicChain.h"

/*
   Kinematic Chain class constructor.
   Sets all the transformation matrices to default values.
*/
KinematicChain::KinematicChain()
{
  this->gBS = xform(0, 0, 0, (141.23), 562, (231.23));
  this->gBC = xform(0, 0, 0, 0, 0, 0);
  // should be replaced with actual coordinates from gantry to base of neck
  this->gCN = xform(0, 0, 0, 0, 0, 0);
  // should be replaced with actual coordinates from base of neck to middle of eyes
  this->gND = xform(0, 0, 0, 0, 0, 0);
  this->gDL = xform(0, 0, 0, (-34.75), 0, 0);
  this->gDR = xform(0, 0, 0, (34.75), 0, 0);

  // these two are the matrices of interest and contain all the information
  // for going from the eyes of the robot to the screen.
  this->gSL = gDL.Inverse() * gND.Inverse() * gCN.Inverse() * gBC.Inverse() * gBS;
  this->gSR = gDR.Inverse() * gND.Inverse() * gCN.Inverse() * gBC.Inverse() * gBS;
}

// Defining the static KinematicChain pointer to null
KinematicChain* KinematicChain::instance_ = NULL;

/*
   Public getInstance() function to return
   the only instance of Kinematic Chain class
*/
KinematicChain* KinematicChain::getInstance()
{
  if (instance_ == NULL) {
    instance_ = new KinematicChain();
  }
  return (instance_);
}

/*
   Function to update the class instance with new stepper
   positions after movement.
*/
void KinematicChain::SetStepperPositions(int x, int y, int z)
{
  this->gBC = xform(0, 0, 0, x, y, z);
}

/*
   Function to update the transformation matrix from the
   screen to the left and right eyes.
*/
void KinematicChain::UpdateTransformation()
{
  this->gSL = gDL.Inverse() * gND.Inverse() * gCN.Inverse() * gBC.Inverse() * gBS;
  this->gSR = gDR.Inverse() * gND.Inverse() * gCN.Inverse() * gBC.Inverse() * gBS;
}

/*
   Function to get the SL transformation matrix
*/
BLA::Matrix<4, 4> KinematicChain::GetSL()
{
  return this->gSL;
}

/*
   Function to get the SR transformation matrix
*/
BLA::Matrix<4, 4> KinematicChain::GetSR()
{
  return this->gSR;
}

/*
   Declaring few helper function for transformation matrix creation.
*/

/*

*/
BLA::Matrix<3, 3> eye()
{
  BLA::Matrix<3, 3> I = {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1
                        };
  return I;
}

BLA::Matrix<3, 3> hat(BLA::Matrix<3> v)
{
  BLA::Matrix<3, 3> vhat;
  vhat.Fill(0);
  vhat(0, 1) = -v(2);
  vhat(0, 2) = v(1);
  vhat(1, 0) = v(2);
  vhat(1, 2) = -v(0);
  vhat(2, 0) = -v(1);
  vhat(2, 1) = v(0);

  return vhat;
}

float norm(BLA::Matrix<3> v)
{
  return sqrt( v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}

BLA::Matrix<3, 3> rodrigues(BLA::Matrix<3> v)
{
  float theta = norm(v);

  if (theta == 0.0)
  {
    return eye();
  }
  else
  {
    BLA::Matrix<3> vnorm = v / norm(v);


    BLA::Matrix<3, 3> vhat = hat(vnorm);

    BLA::Matrix<3, 3> R = eye() + vhat * sin(theta) + vhat * vhat * (1 - cos(theta));

    return R;
  }
}

BLA::Matrix<4, 4> xform(float alpha, float beta, float gamma, float x, float y, float z) {

  BLA::Matrix<3> rotvec = {alpha, beta, gamma};
  BLA::Matrix<3> tvec = {x, y, z};

  BLA::Matrix<3, 3> R = rodrigues(rotvec);

  BLA::Matrix<4, 4> g = {R(0, 0), R(0, 1), R(0, 2), tvec(0),
                         R(1, 0), R(1, 1), R(1, 2), tvec(1),
                         R(2, 0), R(2, 1), R(2, 2), tvec(2),
                         0,      0,      0,       1
                        };
  return g;
}

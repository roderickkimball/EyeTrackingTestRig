#include "KinematicChain.h"

/*
   Kinematic Chain class constructor.
*/
KinematicChain::KinematicChain()
{
}

/*
   Kinematic Chain Class initializer.
   Sets all the transformation matrices to default values.
   All dimensions are in meters.
*/
void KinematicChain::init()
{

  this->gBS = xform(0, 0, 0, (0.14123), 0.562, (0.23123));
  this->gBC = xform(0, 0, 0, 0, 0, 0);
  // should be replaced with actual coordinates from gantry to center of servo
  this->gCY = xform(0, 0, 0, 0, (0.0956), (0.0129));
  // should be replaced with actual coordinates from gantry to base of neck
  this->gYN = xform(0, 0, 0, 0, 0, (0.0672));
  // should be replaced with actual coordinates from base of neck to middle of eyes
  this->gNT = xform(0, 0, 0, 0, 0, 0.127);
  this->gTD = xform(0, 0, 0, (-0.004), (0.0636), (0.0856));
  this->gDL = xform(0, 0, 0, (-0.03475), 0, 0);
  this->gDR = xform(0, 0, 0, (0.03475), 0, 0);

  // declaring the constant transformation matrices for the neck
  // these transformation matrices will not get updated throughout the program.
  this->gO_B1 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 0 * 2 * M_PI / 3),
                 0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 0 * 2 * M_PI / 3),
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0
                };
  this->gO_B2 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 2 * 2 * M_PI / 3),
                 0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 2 * 2 * M_PI / 3),
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0
                };
  this->gO_B3 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 1 * 2 * M_PI / 3),
                 0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 1 * 2 * M_PI / 3),
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0
                };
  this->gPC_P1 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 0 * 2 * M_PI / 3),
                  0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 0 * 2 * M_PI / 3),
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0
                 };
  this->gPC_P2 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 2 * 2 * M_PI / 3),
                  0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 2 * 2 * M_PI / 3),
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0
                 };
  this->gPC_P3 = {1.0, 0.0, 0.0, 0.0635 * cos(M_PI_2 + 1 * 2 * M_PI / 3),
                  0.0, 1.0, 0.0, 0.0635 * sin(M_PI_2 + 1 * 2 * M_PI / 3),
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0
                 };

  this->gB1_P1 = {0};
  this->gB2_P2 = {0};
  this->gB3_P3 = {0};

  // these two are the matrices of interest and contain all the information
  // for going from the eyes of the robot to the screen.
  this->gSL = gDL.Inverse() * gTD.Inverse() * gNT.Inverse() * gYN.Inverse() *  gCY.Inverse() * gBC.Inverse() * gBS;
  this->gSR = gDR.Inverse() * gTD.Inverse() * gNT.Inverse() * gYN.Inverse() *  gCY.Inverse() * gBC.Inverse() * gBS;
}


/*
   Declaring the private helper function for transformation matrix creation.
*/

/*
   Function to create an identity matrix
*/
BLA::Matrix<3, 3> KinematicChain::eye()
{
  BLA::Matrix<3, 3> I = {1, 0, 0,
                         0, 1, 0,
                         0, 0, 1
                        };
  return I;
}

/*
   Function to produce the project matrix or hat of a vector
*/
BLA::Matrix<3, 3> KinematicChain::hat(BLA::Matrix<3> v)
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

/*
   Function to return the norm of a given 3x1 matrix
*/
float KinematicChain::norm(BLA::Matrix<3> v)
{
  return sqrt( v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}

/*
   Function to apply the rodrigues function of a given 3x1 matrix.
*/
BLA::Matrix<3, 3> KinematicChain::rodrigues(BLA::Matrix<3> v)
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

/*
   Function to calculate the outer product of a given matrix
*/
BLA::Matrix<3, 3> KinematicChain::OuterProduct (BLA::Matrix<3> w)
{
  BLA::Matrix<3, 3> product;

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      product(i, j) = w(i) * w(j);
    }
  }

  return product;
}

/*
   Function to calculate the cross product of two given matrices.
*/
BLA::Matrix<3> KinematicChain::CrossProduct (BLA::Matrix<3> w, BLA::Matrix<3> v)
{
  BLA::Matrix<3, 3> w_hat = hat(w);
  return w_hat * v;
}

/*
   Function to raise an exponent to the power of a matrix.
*/
BLA::Matrix<4, 4> KinematicChain::expmXI(BLA::Matrix<6> xI)
{
  BLA::Matrix<3> w_noNorm = {xI(3), xI(4), xI(5)};
  float theta = norm(w_noNorm);
  BLA::Matrix<6> xINorm = xI;

  if (abs(theta) > 10 * FLT_MIN)
  {
    xINorm = xI / theta;

    BLA::Matrix<3> v = {xINorm(0), xINorm(1), xINorm(2)};
    BLA::Matrix<3> w = {xINorm(3), xINorm(4), xINorm(5)};

    BLA::Matrix<3, 3> R = rodrigues(w * theta);

    BLA::Matrix<3, 3> I = eye();

    BLA::Matrix<3, 3> outProduct = OuterProduct(w);
    BLA::Matrix<3> crossProduct = CrossProduct(w, v);

    BLA::Matrix<3> T = ((I - R) * crossProduct) + (outProduct * v * theta);

    BLA::Matrix<4, 4> g = {R(0, 0), R(0, 1), R(0, 2), T(0),
                           R(1, 0), R(1, 1), R(1, 2), T(1),
                           R(2, 0), R(2, 1), R(2, 2), T(2),
                           0,      0,      0,       1
                          };
    //SerialTerminal->println("Translation and Rotation");
    return g;
  }
  else {
    BLA::Matrix<3, 3> R = eye();

    BLA::Matrix<3> v = {xI(0), xI(1), xI(2)};
    BLA::Matrix<3> w = {xI(3), xI(4), xI(5)};

    BLA::Matrix<3> T = (v);



    BLA::Matrix<4, 4> g = {R(0, 0), R(0, 1), R(0, 2), T(0),
                           R(1, 0), R(1, 1), R(1, 2), T(1),
                           R(2, 0), R(2, 1), R(2, 2), T(2),
                           0,      0,      0,       1
                          };
    //SerialTerminal->println("Translation only");
    return g;
  }
}

/*
   Function to create a transformation matrix, given the rotation and translation values.
*/
BLA::Matrix<4, 4> KinematicChain::xform(float alpha, float beta, float gamma, float x, float y, float z) {

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

/*
   Function to update the class instance with new stepper
   positions after movement.
*/
void KinematicChain::SetStepperPositions(float x, float y, float z)
{
  /*
      y is negative here because in all frames of reference
      forward is positive y. since the stepper always move back
      from zero position, we invert the sign of y.
  */
  this->gBC = xform(0, 0, 0, x, -y, z);
}

/*
   Function to update the transformation matrices that go
   from the base of the neck to the top of the neck.
*/
void KinematicChain::UpdateNeckTransformationMatrix(float PhiR, float PhiS, float PhiD, float lSpring)
{
  // getting the matrices that will be raised to the exponent in the next step.
  BLA::Matrix<6> xI01 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  BLA::Matrix<6> xI12 = {0.0, 0.0, 1.0, PhiR / lSpring, 0.0, 0.0};
  BLA::Matrix<6> xI23 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // creating the matrix that will go from the base of the neck to the top of the neck.
  BLA::Matrix<4, 4> g01 = expmXI(xI01 * PhiS);
  BLA::Matrix<4, 4> g02 = expmXI(xI12 * lSpring);
  BLA::Matrix<4, 4> g03 = expmXI(xI23 * (-PhiS));

  this->PrintG(g01);
  this->PrintG(g02);
  this->PrintG(g03);

  this->gYN = expmXI(xI01 * PhiD);
  this->gYN(2, 3) = (0.0672);
  // need to assign the distances once again.
  this->gNT = g01 * g02 * g03;

  this->gB1_P1 = this->gO_B1.Inverse() * this->gNT * this->gPC_P1;
  this->gB2_P2 = this->gO_B2.Inverse() * this->gNT * this->gPC_P2;
  this->gB3_P3 = this->gO_B3.Inverse() * this->gNT * this->gPC_P3;

  //SerialTerminal->println("Updated Neck Transformation Matrices.");
}

/*
   Function to update the transformation matrix from the
   screen to the left and right eyes.
*/
void KinematicChain::UpdateKinematicChain()
{
  this->gSL = gDL.Inverse() * gTD.Inverse() * gNT.Inverse() * gYN.Inverse() * gCY.Inverse() * gBC.Inverse() * gBS;
  this->gSR = gDR.Inverse() * gTD.Inverse() * gNT.Inverse() * gYN.Inverse() * gCY.Inverse() * gBC.Inverse() * gBS;

  Serial.println("SL");
  this->PrintG(this->gSL);
  Serial.println("SR");
  this->PrintG(this->gSR);

  Serial.println("Updated Kinematic Chain");
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
   Function to get the gB1_P1 matrix
*/
BLA::Matrix<4, 4> KinematicChain::GetgB1P1()
{
  return this->gB1_P1;
}

/*
   Function to get the gB1_P1 matrix
*/
BLA::Matrix<4, 4> KinematicChain::GetgB2P2()
{
  return this->gB2_P2;
}

/*
   Function to get the gB1_P1 matrix
*/
BLA::Matrix<4, 4> KinematicChain::GetgB3P3()
{
  return this->gB3_P3;
}

/*
   Function to print a 4x4 BLA matrix passed in
   as argument.
*/
void KinematicChain::PrintG(BLA::Matrix<4, 4> g)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      float x = g(i, j);
      Serial.print(x, 6);
      Serial.print("     ");
    }
    Serial.println(" ");
  }
}

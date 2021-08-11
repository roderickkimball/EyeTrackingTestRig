#include "KinematicChain.h"

/*
   Kinematic Chain class constructor.
*/
KinematicChain::KinematicChain()
{
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

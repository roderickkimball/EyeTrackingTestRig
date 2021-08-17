/*
   This is the header file for the kinematic chain class.
   The class is responsible for storing the transformation matrices of the robot
   joints. It also has public member function to access these private objects and
   helper functions to create the matrices themselves.
*/

#ifndef _KINEMATICCHAIN_H
#define _KINEMATICCHAIN_H

#include <BasicLinearAlgebra.h>
#include "Function.h"
#include <float.h>

/*
   Kinematic chain class made according to singleton pattern.
   This will ensure that only one instance of the class is created at one time
   and the same instance can be used by all the required source files.
*/
class KinematicChain
{
    /*
       Public member functions consist of getter for the instance of class,
       and additional functions to recalculate and update the transformation matrices.
    */
  public:
    KinematicChain();

    /*
      Static Helper function to manipulate and create the transformation matrices
      to be used.
    */
    static BLA::Matrix<3, 3> eye();
    static BLA::Matrix<3, 3> hat(BLA::Matrix<3> v);
    static float norm(BLA::Matrix<3> v);
    static BLA::Matrix<3, 3> rodrigues(BLA::Matrix<3> v);
    static BLA::Matrix<4, 4> expmXI(BLA::Matrix<6> xI);
    static BLA::Matrix<3, 3> OuterProduct(BLA::Matrix<3> w);
    static BLA::Matrix<3> CrossProduct(BLA::Matrix<3> w, BLA::Matrix<3> v);
    static BLA::Matrix<4, 4> xform(float alpha, float beta, float gamma, float x, float y, float z);

    static void PrintG(BLA::Matrix<4, 4> g);
};

#endif

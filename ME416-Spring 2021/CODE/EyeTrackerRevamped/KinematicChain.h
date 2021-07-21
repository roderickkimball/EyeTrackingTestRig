/*
   This is the header file for the kinematic chain class.
   The class is responsible for storing the transformation matrices of the robot
   joints. It also has public member function to access these private objects and
   helper functions to create the matrices themselves.
*/

#ifndef _KINEMATICCHAIN_H
#define _KINEMATICCHAIN_H

#include <BasicLinearAlgebra.h>

/*
   Kinematic chain class made according to singleton pattern.
   This will ensure that only one instance of the class is created at one time
   and the same instance can be used by all the required source files.
*/
class KinematicChain
{
    /*
       Private members consist of constructor so object can't
       be created outside the class and a pointer to the instance of the class.
    */
  private:
    static KinematicChain* instance_;
    KinematicChain();
    KinematicChain(const KinematicChain&) {};
    KinematicChain& operator=(const KinematicChain&) {};

    /*
       Private helper function to manipulate and create the transformation matrices
       to be used.
    */
    BLA::Matrix<3, 3> eye();
    BLA::Matrix<3, 3> hat(BLA::Matrix<3> v);
    float norm(BLA::Matrix<3> v);
    BLA::Matrix<3, 3> rodrigues(BLA::Matrix<3> v);
    BLA::Matrix<4, 4> expmXI(BLA::Matrix<6> xI);
    BLA::Matrix<3, 3> OuterProduct(BLA::Matrix<3> w);
    BLA::Matrix<3> CrossProduct(BLA::Matrix<3> w, BLA::Matrix<3> v);
    BLA::Matrix<4, 4> xform(float alpha, float beta, float gamma, float x, float y, float z);

    /*
       Protected members are the transformation matrices themselves

      S - will be the screen frame of reference with the zero coordinate at the center of the screen,
      X will be to the right, Z up, and Y into the screen
      B - will be the origin frame of reference with the zero coordinate at the position where the
      stages of the 3-axis shoulder steppers hit the limit switches or zero position
      C - will be the origin frame of reference located at the center of the Z-axis stage,
      that the neck and eye mechanisms are mounted to.
      N - will be the frame of reference located at the base of the neck of the robot.
      T - will be the frame of reference located at the center of the top of the neck.
      D - will be the frame of reference at the center of the eye mechanisms,
      with Z up, X to the right and Y along the axis of the straight 90-degree eyes
      L - will be the left eye's frame of reference with the origin at the center of rotation
      and is aligned with the D frame when the 90-degree angle is commanded
      R - will be the right eye's frame of reference with the origin at the center of rotation
      and is aligned with D frame when the 90-degree angle is commanded
    */
  protected:
    BLA::Matrix<4, 4> gBS, gBC, gCN, gNT, gTD, gDL, gDR, gSL, gSR;

    /*
       Now we define the transformation matrices for the neck.
       O - signifies the origin point of the neck at the middle of the base.
       B1 - front point at the base; B2 - back right point at the base; B3 - back left point at the base;
       P1 - front point at the top; P2 - back right point; P3 - back left point; PC - center point at the top
    */
    BLA::Matrix<4, 4> gO_B1, gO_B2, gO_B3, gPC_P1, gPC_P2, gPC_P3;
    BLA::Matrix<4, 4> gB1_P1, gB2_P2, gB3_P3; 


    /*
       Public member functions consist of getter for the instance of class,
       and additional functions to recalculate and update the transformation matrices.
    */
  public:
    static KinematicChain* getInstance();

    void SetStepperPositions(int x, int y, int z);
    
    void UpdateNeckTransformationMatrix(float PhiR, float PhiS, float lSpring);
    
    void UpdateKinematicChain();
    BLA::Matrix<4, 4> GetSL();
    BLA::Matrix<4, 4> GetSR();
    BLA::Matrix<4, 4> GetgB1P1();
    BLA::Matrix<4, 4> GetgB2P2();
    BLA::Matrix<4, 4> GetgB3P3();
};

#endif

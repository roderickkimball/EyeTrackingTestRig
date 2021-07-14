#ifndef _KINEMATICCHAIN_H
#define _KINEMATICCHAIN_H

#include <BasicLinearAlgebra.h>

class KinematicChain
{
  private:
    static KinematicChain* instance_;
    KinematicChain();
    KinematicChain(const KinematicChain&);
    KinematicChain& operator=(const KinematicChain&);

  protected: 
    BLA::Matrix<4,4> gBS, gBC, gCN, gND, gDL, gDR, gSL, gSR;
    

  public:
    static KinematicChain* getInstance();

    void SetStepperPositions(int x, int y, int z);
    void UpdateTransformation();
    BLA::Matrix<4,4> GetSL();
    BLA::Matrix<4,4> GetSR();
};

/*
 * Declaring few helper function for the kinematic chain transformation.
 */

/*
 *  
 */
extern BLA::Matrix<3,3> eye();
 
extern BLA::Matrix<3,3> hat(BLA::Matrix<3> v);
 
extern float norm(BLA::Matrix<3> v);

extern BLA::Matrix<3,3> rodrigues(BLA::Matrix<3> v);

extern BLA::Matrix<4, 4> xform(float alpha, float beta, float gamma, float x, float y, float z);

#endif

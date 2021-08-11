#include "NeckClass.h"

NeckMotor neckCalibrationMotor = front;

// Stepper Configuration variables.
float spoolRadius = 0.0071;                      // radius of the spool that holds the thread
float spoolCircumference = 2.0 * M_PI * spoolRadius; // circumference of the spool
float mmPerRevs = spoolCircumference;            // mm of thread that each revolution of spool will loosen or tighten
float stepsPerRev = 3838;                        // number of steps in one revolution for the neck steppers
float mmPerStep = mmPerRevs / stepsPerRev;       // mm of thread released in one step

// The known stepper pos after calibration. = (known cable length)/ mmPerStep
// 0.127 is the length of calibration stick and hence the known cable length.
float knownStepperPos = 0.127 / mmPerStep;

// number of microseconds per degree for yaw servo
float yawMicroSecondsPerDegree = 2000.00 / 270.00 * 2.4;

/*
   Neck Class constructor.
*/
RobotNeck::RobotNeck()
{
}

/*
   Neck Class destructor to clear pointer objects.
*/
RobotNeck::~RobotNeck()
{
  delete[] this->frontStepper;
  delete[] this->backLeftStepper;
  delete[] this->backRightStepper;
}

/*
   Function to initialize the Neck objects. Function also sets the maxspeed
   and acceleration for the stepper objects as well.
*/
void RobotNeck::init()
{
  this->frontStepper = new AccelStepper(AccelStepper::DRIVER, frontPulse, frontDir);
  this->backLeftStepper = new AccelStepper(AccelStepper::DRIVER, backLeftPulse, backLeftDir);
  this->backRightStepper = new AccelStepper(AccelStepper::DRIVER, backRightPulse, backRightDir);

  this->m3FrontStepper.init(frontDir, frontPulse);
  this->m3BackRightStepper.init(backRightDir, backRightPulse);
  this->m3BackLeftStepper.init(backLeftDir, backLeftPulse);

  this->m3FrontStepper.SetSpeed(1000);
  this->m3BackRightStepper.SetSpeed(1000);
  this->m3BackLeftStepper.SetSpeed(1000);

  this->m3FrontStepper.SetCurrentPosition(knownStepperPos);
  this->m3BackRightStepper.SetCurrentPosition(knownStepperPos);
  this->m3BackLeftStepper.SetCurrentPosition(knownStepperPos);

  this->neckServo.attach(neckServoPin);

  this->lastPhiR = 0;
  this->lastPhiS = 0;
  this->lastPhiD = 0;

  /*
     Giving initial values to the neck transformation matrices.
  */
  this->gCY = KinematicChain::xform(0, 0, 0, 0, (0.0956), (0.0129));
  // should be replaced with actual coordinates from gantry to base of neck
  this->gYN = KinematicChain::xform(0, 0, 0, 0, 0, (0.0672));
  // should be replaced with actual coordinates from base of neck to middle of eyes
  this->gNT = KinematicChain::xform(0, 0, 0, 0, 0, 0.127);

  this->gTC = this->gNT.Inverse() * this->gYN.Inverse() * this->gCY.Inverse();


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

  /*
    Length of the spring in meters. Gets updated after MoveToCalState()
    It is 127mm before calibration.
  */
  this->lSpring = 0.127;
}

/*
   Function to calculate the transformation matrices that go from the bottom plate of the neck
   to the top plate of the neck. This function must be called whenever the neck angles are set.
*/
void RobotNeck::updateNeckTransformation()
{
  // getting the matrices that will be raised to the exponent in the next step.
  BLA::Matrix<6> xI01 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  BLA::Matrix<6> xI12 = {0.0, 0.0, 1.0, this->lastPhiR / this->lSpring, 0.0, 0.0};
  BLA::Matrix<6> xI23 = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // creating the matrix that will go from the base of the neck to the top of the neck.
  BLA::Matrix<4, 4> g01 = KinematicChain::expmXI(xI01 * this->lastPhiS);
  BLA::Matrix<4, 4> g02 = KinematicChain::expmXI(xI12 * this->lSpring);
  BLA::Matrix<4, 4> g03 = KinematicChain::expmXI(xI23 * -(this->lastPhiS));

  KinematicChain::PrintG(g01);
  KinematicChain::PrintG(g02);
  KinematicChain::PrintG(g03);

  this->gYN = KinematicChain::expmXI(xI01 * this->lastPhiD);
  this->gYN(2, 3) = (0.0672);
  // need to assign the distances once again.
  this->gNT = g01 * g02 * g03;

  this->gB1_P1 = this->gO_B1.Inverse() * this->gNT * this->gPC_P1;
  this->gB2_P2 = this->gO_B2.Inverse() * this->gNT * this->gPC_P2;
  this->gB3_P3 = this->gO_B3.Inverse() * this->gNT * this->gPC_P3;
}

/*
   Returns the inverse transformation matrix that goes from the top of the neck to the center
   of the z axis stepper stage
*/
BLA::Matrix<4, 4> RobotNeck::GetInverseNeckTransformation()
{
  this->gTC = this->gNT.Inverse() * this->gYN.Inverse() * this->gCY.Inverse();
  return this->gTC;
}

/*
   Function to update the neck transformation matrices based on the last PhiR, PhiS and PhiD values.
   The transformation matrices are then used to determine what the current positions of each of the
   neck steppers would be.
*/
void RobotNeck::SetLastNeckPosition()
{
  this->updateNeckTransformation();

  float frontCableLength = sqrt(sq(gB1_P1(0, 3)) + sq(gB1_P1(1, 3)) + sq(gB1_P1(2, 3)));
  float backRightCableLength = sqrt(sq(gB2_P2(0, 3)) + sq(gB2_P2(1, 3)) + sq(gB2_P2(2, 3)));
  float backLeftCableLength = sqrt(sq(gB3_P3(0, 3)) + sq(gB3_P3(1, 3)) + sq(gB3_P3(2, 3)));

  this->m3FrontStepper.SetCurrentPosition(frontCableLength / mmPerStep);

  this->m3BackRightStepper.SetCurrentPosition(backRightCableLength / mmPerStep);

  this->m3BackLeftStepper.SetCurrentPosition(backLeftCableLength / mmPerStep);

  this->neckServo.writeMicroseconds(this->neckServoCenter + this->lastPhiD * (180 / M_PI) * yawMicroSecondsPerDegree);

}

/*
   Function to get the last commanded neck angles based on the string argument passed
   that denotes which angle is to be returned.
*/
float RobotNeck::GetLastNeckPosition(String whichAngle)
{
  if (whichAngle == "PhiR")
  {
    return this->lastPhiR;
  }
  else if (whichAngle == "PhiS")
  {
    return this->lastPhiS;
  }
  else if (whichAngle == "PhiD")
  {
    return this->lastPhiD;
  }
}

/*
   Function to move the neck manually. The function takes in two parameters that determine
   pitch and roll of the neck.
   X and Y should be scaled between -1 and 1.
   Z should be scaled between 0 and 1.
*/
void RobotNeck::MoveNeckManually(float x, float y, float z)
{
  this->lastPhiR = x;
  this->lastPhiS = y;
  this->lastPhiD = z;

  this->updateNeckTransformation();

  float frontCableLength = sqrt(sq(this->gB1_P1(0, 3)) + sq(this->gB1_P1(1, 3)) + sq(this->gB1_P1(2, 3)));
  float backRightCableLength = sqrt(sq(this->gB2_P2(0, 3)) + sq(this->gB2_P2(1, 3)) + sq(this->gB2_P2(2, 3)));
  float backLeftCableLength = sqrt(sq(this->gB3_P3(0, 3)) + sq(this->gB3_P3(1, 3)) + sq(this->gB3_P3(2, 3)));

  this->m3FrontStepper.MoveTo(frontCableLength / mmPerStep);

  this->m3BackRightStepper.MoveTo(backRightCableLength / mmPerStep);

  this->m3BackLeftStepper.MoveTo(backLeftCableLength / mmPerStep);

  this->neckServo.writeMicroseconds(this->neckServoCenter + this->lastPhiD * (180 / M_PI) * yawMicroSecondsPerDegree);
}

/*
   Function to run the stepper motors to the desired set position.
   This function must be called everytime the move neck manually function is called.
*/
void RobotNeck::RunSteppers()
{
  unsigned long startTime = micros();
  unsigned long frontNext = startTime + ((1L * 1000L * 1000L) / this->m3FrontStepper.GetSpeed());
  unsigned long backRightNext = startTime + ((1L * 1000L * 1000L) / this->m3BackRightStepper.GetSpeed());
  unsigned long backLeftNext = startTime + ((1L * 1000L * 1000L) / this->m3BackLeftStepper.GetSpeed());

  while (!(m3FrontStepper.IsReached()) || !(m3BackRightStepper.IsReached()) || !(m3BackLeftStepper.IsReached()))
  {
    unsigned long currentTime = micros();

    if (currentTime >= frontNext)
    {
      this->m3FrontStepper.RunStepper();

      //SerialTerminal->print("front: ");
      Serial.println(this->m3FrontStepper.CurrentPosition());
      frontNext = currentTime + ((1L * 1000L * 1000L) / this->m3FrontStepper.GetSpeed());
    }
    if (currentTime >= backRightNext)
    {
      this->m3BackRightStepper.RunStepper();

      //SerialTerminal->print("Right: ");
      Serial.println(this->m3BackRightStepper.CurrentPosition());
      backRightNext = currentTime + ((1L * 1000L * 1000L) / this->m3BackRightStepper.GetSpeed());
    }
    if (currentTime >= backLeftNext)
    {
      this->m3BackLeftStepper.RunStepper();

      //SerialTerminal->print("left: ");
      Serial.println(this->m3BackLeftStepper.CurrentPosition());
      backLeftNext = currentTime + ((1L * 1000L * 1000L) / this->m3BackLeftStepper.GetSpeed());
    }
  }
}

/*
   Function to write the current neck stepper positions to the EEPROM.
   This function can save the positions to either the calibration variables in the PROM
   or the last stepper position variables in the PROM based on the character input.
   'c' for calibration. 'l' for last stepper.
*/
void RobotNeck::WriteNeckPositionToProm()
{
  PromAddress eeAddress = LastPhiR;

  Serial.println(this->lastPhiR);
  EEPROM.put(eeAddress, this->lastPhiR);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  Serial.println(this->lastPhiS);
  EEPROM.put(eeAddress, this->lastPhiS);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.put(eeAddress, this->lastPhiD);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.put(eeAddress, this->neckServoCenter);
  delay(100);

  //SerialTerminal->println("Neck variables written!");
}

/*
   Function to read either the calibration stepper positions
   or the last neck stepper positions from the EEPROM. The character input into the function
   determines which variables are read.
*/
void RobotNeck::ReadNeckPositionFromProm()
{
  PromAddress eeAddress = LastPhiR;
  float angle = 0.0;
  long int stepperPosition = 0;

  // reading last commanded PhiR, PhiS, PhiD values
  EEPROM.get(eeAddress, angle);
  this->lastPhiR = angle;
  Serial.println(this->lastPhiR);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, angle);
  this->lastPhiS = angle;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, angle);
  this->lastPhiD = angle;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  // reading neck servo center location and sending to center.
  EEPROM.get(eeAddress, stepperPosition);
  this->neckServoCenter = stepperPosition;

  //SerialTerminal->println("Neck variables read!");
}

/*
   Function to calibrate the neck. Calibration sequence attempts to
   level the neck to a known height indicated by the knownStepperPos variable
   and the calibration stick.
*/
void RobotNeck::CalibrateNeck(char neckCalCommand)
{
  if (neckCalCommand == '1' || neckCalCommand == '2' || neckCalCommand == '3' || neckCalCommand == '4') {
    int motorChoice = neckCalCommand - '0';
    neckCalibrationMotor = static_cast<NeckMotor>(motorChoice);
  }
  else if (neckCalCommand == 'z') {
    this->m3FrontStepper.SetCurrentPosition(knownStepperPos);
    this->m3BackRightStepper.SetCurrentPosition(knownStepperPos);
    this->m3BackLeftStepper.SetCurrentPosition(knownStepperPos);
  }

  switch (neckCalibrationMotor) {
    case (front): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->m3FrontStepper.MoveTo(this->m3FrontStepper.CurrentPosition() + 50);
        }
        if (neckCalCommand == 'd') {
          this->m3FrontStepper.MoveTo(this->m3FrontStepper.CurrentPosition() - 50);
        }
        break;
      }
    case (backRight): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->m3BackRightStepper.MoveTo(this->m3BackRightStepper.CurrentPosition() + 50);
        }
        if (neckCalCommand == 'd') {
          this->m3BackRightStepper.MoveTo(this->m3BackRightStepper.CurrentPosition() - 50);
        }
        break;
      }
    case (backLeft): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->m3BackLeftStepper.MoveTo(this->m3BackLeftStepper.CurrentPosition() + 50);
        }
        if (neckCalCommand == 'd') {
          this->m3BackLeftStepper.MoveTo(this->m3BackLeftStepper.CurrentPosition() - 50);
        }
        break;
      }
    case (yawServo): {
        // 'u' will twist to right and 'd' will twist to left
        if (neckCalCommand == 'u') {
          this->neckServoCenter += 5;
          this->neckServo.writeMicroseconds(this->neckServoCenter);
        }
        if (neckCalCommand == 'd') {
          this->neckServoCenter -= 5;
          this->neckServo.writeMicroseconds(this->neckServoCenter);
        }
        break;
      }

    default: break;
  }
  this->RunSteppers();

  this->m3FrontStepper.SetCurrentPosition(knownStepperPos);
  this->m3BackRightStepper.SetCurrentPosition(knownStepperPos);
  this->m3BackLeftStepper.SetCurrentPosition(knownStepperPos);
}

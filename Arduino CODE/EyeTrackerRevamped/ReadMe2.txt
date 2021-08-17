CODE NOTES:

• The current class structure ensures that the different subsystems of the robot are encapsulated into their own objects and de-coupled from each other. 
• This sort of class structures enables modularity in software design. What this means is that if we want to swap out a particular subsystem for a more modified subsystem design, then one can do that by making changes solely to that subsystem's header and source files, along with minimal to zero changes to the main Robot class.
• The rest of this document explains how to do so, assuming that we want to swap out the neck subsystem. It must be noted that the same step sequence could be applied to swap out a different subsystem as well. 

Swapping the neck:

• Encapsulate the variables, frames of references, transformation matrices between these frames of references and all the functionalities pertaining to the neck into a class with a header (.h) and source (.cpp) file.
• Delete the existing RobotNeck.h and RobotNeck.cpp files and replace them with the new header and source files. 
• Make sure that the new Neck class has a method to return the inverse transformation matrix of the entire subsystem. For instance, in the existing code this would be the transformation matrix that goes from the top of the neck plate to the center of the z-axis stepper stage. 
• This method must be named GetInverseNeckTransformation() and must return a 4x4 matrix to ensure minimal changes need to be made.
• Replace the existing Neck object with the new Neck class object in Robot.h and subsequently in all the places where this object is called. This should ideally be as simple as finding and replacing the robotNeck object that currently exists with the new class object.
• The init(), runRobotState() and runNeckCalibrationState() functions in Robot.h must also be updated with the methods from the new neck class that perform the same functionalities. Note: The runRobotState() function must only be changed in the sections pertaining to manual neck motion.
• The new neck class must have functions that read and write variables pertaining to neck motion to the EEPROM. If the variables are different from the ones currently being stored, then those variables must be enumerated in PromAddress enumeration in Function.h.
• The method names for reading and writing neck variables to the EEPROM must be updated wherever called inside Robot.cpp.
• Compile, Upload and you should have successfully switched subsystems!
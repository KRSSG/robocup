# tactics
STP Tactics for Robocup SSL by KRSSG

## Instructions for adding new tactics:

* create new files tXXX.hpp and tXXX.cpp in include/tactics and src/ folders respectively
* inherit class TXXX from Tactic (use Strategy namespace)
* implement all the virtual functions of Tactic. Some function prototypes have changed from old API, make sure they are corrected.
* in tXXX.hpp, add the line 
  ```
  REGISTER_TACTIC(TXXX)
  ```
  outside the TXXX class, but inside Strategy namespace (see tPosition.hpp on how its done)
* in CMakeLists.txt, append tXXX.cpp to the 'tacticsSrc' variable



Instructions for testing tactic will be given in package `ssl_robot`
Mascot_testing.py will generate X and Z coordinates for any message that is passed into it. 
The desired string message must be typed into the "message" variable in the script.
The script uses Eyetracker_Keyboard.csv file, which contains the coordinates for all the keyboard letters.
If the keyboard coordinate values change from the ones given in the .csv file, then these should be updated using the find_coordinates state of the robot. This state of the robot will provide you with the coordinates of gaze of the robot.

The python script will write the coordinate data to a GFG.csv file. 
The coordinate values will need to be copied from this .csv file and pasted into the autoCoordinates variable inside the arduino code.

HAPPY TYPING!
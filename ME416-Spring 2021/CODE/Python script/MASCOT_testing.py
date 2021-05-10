import pandas as pd
import os
import csv

_location_ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__))
)


keyboard_coordinates = pd.read_csv(os.path.join(_location_, 'Eyetracker_Keyboard.csv'))
keyboard_coordinates.to_dict()

def convert_letters_to_coordinates(message):
    message = message.upper()
    coordinateList = []

    for letter in message:
        index = keyboard_coordinates[keyboard_coordinates['Letter'] == letter]
        x_pos = index.iloc[0]['X']
        z_pos = index.iloc[0]['Z']

        x_pos = str(x_pos) + ','
        z_pos = str(z_pos) + ','

        coordinateList.append((x_pos,z_pos))

    return coordinateList

message = "Life is difficult. Not just for me or other ALS patients. Life is difficult for everyone. Finding ways to make life meaningful and purposeful and rewarding, doing the activities that you love and spending time with the people that you love, I think that is the meaning of this human experience."

coordinateList = convert_letters_to_coordinates(message)

print(coordinateList)

df = pd.DataFrame(coordinateList)
df.to_csv('GFG.csv')



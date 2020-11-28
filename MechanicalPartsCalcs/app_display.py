from math import pi
import tkinter as tk
from mech_calculator import *
"""
This program calculates the diameter and desired angles for a GT timing pulley.
To ease the design process in CAD software.
"""

def menuInterface():

    print("Welcome to a part calculator for your SolidWorks schematics.")
    print("Remember to enter the dimension in mm.")
    print("Select an option:")
    print("")
    print("1.- GT2Pulley")
    print("2.- Gears")
    print("")
    print("Write your option like in the menu.")
    return input()

if __name__ == "__main__":
    
    """
    if(menuInterface().lower() == "gt2pulley"):
        print(GT2Pulley())
    else:
        print("Nothing to do.")
    """

    # Create the window for the GUI
    app = tk.Tk()
    # Name the window app
    app.title("Mechanical Parts Calculator")
    # Set default size
    app.geometry('800x600')
    # Set the sacred rites
    label = tk.Label(app, text="Praise the machine").pack(side="bottom")
    # Frame inside the main window
    frame = tk.Frame(app)
    frame.pack(side="bottom")

    # Add scrollbar


    # Calculate button
    button = tk.Button(frame, text="Calculate", command= app.destroy)
    button.pack()
     

    
    # Run app
    app.mainloop()


    
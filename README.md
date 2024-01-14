#SOFA_random_Gravity_exportObj

SOFA_random_Gravity_exportObj is a project that utilizes the SOFA framework to simulate liver model behavior under randomly varying gravity conditions and exports the results as object files. This project can be used for research or educational purposes in the field of biomechanics.
Features

    Simulates liver model behavior under a randomly changing gravity environment.
    Generates visual models of the liver and associated structures (portal, veins, tumors, etc.).
    Exports 3D models as OBJ files at each step of the simulation.

How to Use

    Install the SOFA framework.
    Clone or download this repository.
    Run the script to start the simulation.

Code Explanation

    Imports Sofa and other necessary modules.
    Sets up mesh files and export paths.
    Defines the ManageGravity class to apply random gravity in the simulation.
    Creates the liver model and its visual representation, and adds collision models in the createScene function.
    Controls initialization and execution of the simulation in the main function.

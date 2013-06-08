Here, we briefly summarize the purpose of each file, and describe the relations
between the files. See file contents for more information.

All .cpp files become executables with the same name.

Authors: Chris Dembia & Sean Sketch

Modeling: modeling.cpp
===============================================================================
Prints two cat models, one that cannot twist its body,
flippinfelines_hunch_wag.osim, and one that CAN twist its body and also has
legs, flippinfelines_huch_wag_twist_legs.osim.


Optimization
===============================================================================

FelisCatusOptimizerSystem.h
---------------------------
Creates a subclass of SimTK::OptimizerSystem whose objective is to achieve a
cat flip, which does so by running forward simulations on a given cat model.
The parameters are the time histories of the actuation of the cat model.

There's another class in this file, FelisCatusOptimizerTool, whose purpose is
to define inputs/settings for the OptimizerSystem by parsing an XML input file.
The use of FelisCatusOptimizerTool is modeled roughly after the use of
OpenSim::AbstractTool subclasses.

optimize.cpp
------------
Given an XML input file, creates a FelisCatusOptimizerTool, and uses it to
instantiate a FelisCatusOptimizerSystem and a FelissCatusOptimizer, and run an
optimization. Manages some of the file output.

optimizer_tool_template.cpp
---------------------------
Prints a template XML file that can be passed to the 'optimize' executable.

initial_parameters.xml
--------------------------------------
An example of a FunctionSet serialization that can be used to set initial
parameters in the optimization. NOTE: This is NOT a FelisCatusOptimizerTool
serialization/'optimize' input file. May need to be augmented depending on the
number of spline points used in the optimization ('num_optim_spline_points'),
and the actuators in the model being optimized.


Scripts
=======

plotActuation.py
----------------
An OpenSim GUI script that plots the functions in a given FunctionSet
serialization. Useful for visualizing intermediate ('best_yet') or final
('optimized' or 'last') results of an optimization.


Input files
===============================================================================

counterrotation_setup.xml
--------------------------------------------------------
A setup file for the 'optimize' executable that will achieve a flip for
the flippinfelines_huch_wag.osim model via the counter-rotation flip mechanism.

counterrotation_variableinertia_setup.xml,
counterrotation_variableinertia_initial_parameters.xml
------------------------------------------------------
A setup file, and the initial parameters file it requires, for the 'optimize'
executable that will achieve a flip for the
flippinfelines_huch_wag_twist_legs.osim model via both the counter-rotation and
variable-inertia flip mechanisms.

Authors: Chris Dembia & Sean Sketch

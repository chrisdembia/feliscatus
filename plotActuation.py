# An OpenSim GUI script that creates a plotter window, and plots in it all the
# actuation curves. The actuation curves are obtained via a serialized file
# that contains a FunctionSet of all the Function's in the
# PrescribedController.
#
# The script is for use with the FelisCatus project for ME 485 at Stanford
# University.
# Note that this script requires a version of OpenSim v3.1.WW or higher;
# earlier versions did not have FunctionSet exposed.
#
# Authors: Chris Dembia, Sean Sketch
# Date: 19 May 2013

import org.opensim.utils as utils

# Prompt user for the file containing FunctionSet.
filepath = utils.FileUtils.getInstance().browseForFilename(".xml",
        "Select a file containing a FunctionSet.")
        
# Load the FunctionSet.
fset = modeling.FunctionSet(filepath)

plot = createPlotterPanel("Flip Actuation")

crv = list()
# Create a curve for each function in the set.
for i in range(fset.getSize()):
    crv.append(addFunctionCurve(plot, fset.get(i)))

plot.setYAxisLabel("Force (N) / Torque (Nm)")
plot.setXAxisLabel("time (s)")

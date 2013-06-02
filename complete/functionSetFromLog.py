# An OpenSim GUI script that reads in a FelisCatusOptimizerSystem log
# to produce a FunctionSet of the actuation at a prescribed objective call number.

# Authors: Chris Dembia, Sean Sketch
# Date: 24 May 2013

import org.opensim.utils as utils

# Prompt user for the log.
filepath = utils.FileUtils.getInstance().browseForFilename(".txt",
        "Select the desired FelisCatusOptimizerSystem log.")
		
# TODO use a GUI tool.
objectiveCallNumber = 15865

f = open(filepath, 'r')

for line in f:
    # The file is space-delimited.
    row = line.split(' ')
    if row[0] == 'objective_calls':
        header_row = row
        # number of spline points per 
    if row[0] == str(objectiveCallNumber):
        print row
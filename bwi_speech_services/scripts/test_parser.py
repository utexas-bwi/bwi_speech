#!/usr/bin/python

import sys

#TODO Change if necessary. 
#Adds nlu_pipeline src folder in order to import modules from it. 
#nlu_pipeline_path = '/home/rcorona/catkin_ws/src/bwi_speech/NLL/CkyParser/src/'
nlu_pipeline_path = '/scratch/cluster/rcorona/catkin_ws/src/bwi_speech/NLL/CkyParser/src/' #For Bender
sys.path.append(nlu_pipeline_path)

#TODO Change if necessary. 
#Path to CKYParser
#parser_path = '/home/rcorona/catkin_ws/src/bwi_speech/src/parser.cky'
parser_path = '/scratch/cluster/rcorona/catkin_ws/src/bwi_speech/src/arm_parser.pckl'

#Nlu pipeline modules.
try:
    import CKYParser
    from CKYParser import count_ccg_productions
    from utils import *
    from Ontology import Ontology

    #grounder_path = '/home/justin/UT/Research/Code/dialog_active_learning/src'
    #sys.path.append(grounder_path)

except ImportError, e:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    print 'Error details: ' + str(e)
    sys.exit()

parser = load_obj_general(parser_path)

parser.print_parameters()

while True:
    response = raw_input('Input:')
    semantic_node = parser.most_likely_cky_parse(response).next()[0].node 
    parse = parser.print_parse(semantic_node)

    print "PARSE: " + parse



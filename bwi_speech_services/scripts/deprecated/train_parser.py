#!/usr/bin/env python
__author__ = 'jesse'

import sys

parser_path = '/scratch/cluster/rcorona/catkin_ws/src/bwi_speech/NLL/CkyParser/src/'
sys.path.append(parser_path)

import Ontology
import Lexicon
import CKYParser
from utils import *

print "reading in Ontology"
ont = Ontology.Ontology(sys.argv[1])
print "predicates: " + str(ont.preds)
print "types: " + str(ont.types)
print "entries: " + str(ont.entries)

print "reading in Lexicon"
lex = Lexicon.Lexicon(ont, sys.argv[2])
print "surface forms: " + str(lex.surface_forms)
print "categories: " + str(lex.categories)
print "semantic forms: " + str(lex.semantic_forms)
print "entries: " + str(lex.entries)

parser = CKYParser.CKYParser(ont, lex, use_language_model=True)
# Set parser hyperparams to best known values for training
parser.max_multiword_expression = 4  # max span of a multi-word expression to be considered during tokenization
parser.max_new_senses_per_utterance = 3  # max number of new word senses that can be induced on a training example
parser.max_cky_trees_per_token_sequence_beam = 100  # for tokenization of an utterance, max cky trees considered
parser.max_hypothesis_categories_for_unknown_token_beam = 3  # for unknown token, max syntax categories tried
d = parser.read_in_paired_utterance_semantics(sys.argv[3])
converged = parser.train_learner_on_semantic_forms(d, 10, reranker_beam=10)

if not converged:
    print 'Parser not converged!'
    #raise AssertionError("Training failed to converge to correct values.")

save_obj_general(parser, 'arm_parser.pckl')

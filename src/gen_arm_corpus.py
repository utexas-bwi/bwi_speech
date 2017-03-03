#!/usr/bin/python

import sys

if __name__ == '__main__':
    train_file = open('parser_train.txt', 'w')
    templates = open('arm_templates.txt', 'r')

    items = [
             ['bevo', ['bevo', 'plush toy', 
                       'stuffed animal', 'longhorn',
                       'cow', 'toy']],

             ['crayon', ['crayon', 'pink crayon', 'cylinder', 
                         'pink cylinder', 'crayola', 'pink crayola']]
            ]

    actions = [
               ['grasp', ['grasp', 'grab', 'hold']],
               ['lift', ['lift', 'raise', 'elevate']],
               ['pickup', ['pick up']],
               ['place', ['place', 'put down', 'drop']],
               ['handover', ['hand over', 'hand me', 'give me', 'hand them', 'give them']]
              ]

    #Generate a phrase for every possible parameterization. 
    for template in templates: 

        #All items. 
        for item_grounding, item_surface_forms in items: 

            #All surface forms for this item. 
            for item_surface_form in item_surface_forms:

                #All actions. 
                for action_grounding, action_surface_forms in actions: 

                    #All surface forms for this action. 
                    for action_surface_form in action_surface_forms: 
                        phrase = template.strip().replace('<I>', item_surface_form).replace('<A>', action_surface_form)
                
                        semantic_form = 'M : ' + action_grounding + '(' + item_grounding + ')'

                        train_file.write(phrase + '\n' + semantic_form + '\n\n')

    #Close template file to open gripper one. 

    #Templates for opening of gripper. 
    surface_forms = ['hand', 'gripper', 'fingers', 'claws', 'claw']
    templates = open('open_gripper_templates.txt', 'r')
    
    #Generate a phrase for every combination as done above. 
    for template in templates:

        for surface_form in surface_forms: 
            
            phrase = template.strip().replace('<H>', surface_form)
            semantic_form = 'M : open(gripper)'

            train_file.write(phrase + '\n' + semantic_form + '\n\n')

            
            

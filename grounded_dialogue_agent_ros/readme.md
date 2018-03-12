# grounded_dialogue_agent_ros

Provides a ROS wrapper for a natural language interface for providing commands to the robot. Implemented via calls into [Jesse Thomason's]() parser and dialogue agents interactive language learning dialogue

## Usage

Place copies of TSP and grounded_dialogue_agent in the parent directory of this package. Ensure that they have the necessary files and configuration in place.

    rosrun grounded_dialogue_agent_ros conduct_dialogue_as
    
To test calling the action

    rosrun actionlib axclient.py /conduct_dialogue grounded_dialogue_agent_ros/ConductDialogue
    
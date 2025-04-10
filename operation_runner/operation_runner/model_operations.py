from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from py_ctrl.model.operation import Operation, Transition
from py_ctrl.predicates.state import State
import py_ctrl.predicates.guards
import py_ctrl.predicates.actions
from py_ctrl.predicates.guards import AlwaysTrue, Guard, And
from py_ctrl.predicates.guards import AlwaysFalse

@dataclass
class Model(object):
    #state and operations
    initial_state: State #we want to have a state
    operations: Dict[str, Operation] #we want a dictionary with the operations ({nameOfOperation: operation})
    transitions: List[Transition] #we need a list of all the transitions

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)

g = py_ctrl.predicates.guards.from_str #allows us to define guards
a = py_ctrl.predicates.actions.from_str#allows us to define what actions we want to happen

def theModel() -> Model:
    initial_state = State(
        #add all the state variables you want here, for simplistic reasons I'll only add a few.
                
        AGV1_allowedToMoveLocation = True,
        AGV1_pos = 'pos1',
        finishedMove = True,
        moveAVG1 = False,
    )
    
    ops = {} # we store all of our operations in a dict
    
    for nr in range(1,5):
        for place in ['Warehouse', 'Pos1']:
            ops[f"moveAGV{nr}To{place}"] = Operation(
                name = f"moveAGV{nr}To{place}", #the name of the operation
                precondition=Transition("pre",
                    g(f"AGV{nr}_allowedToMoveLocation && AGV{nr}_pos != {place}"),                                          
                    a(f"!AGV{nr}_allowedToMoveLocation, moveAGV{nr}")
                ),
                postcondition=Transition("post",
                    g(f"AGV{nr}_pos == {place}"), 
                    a(f"AGV{nr}_allowedToMoveLocation")#AGVPose <- warehouse
                ),
                
                effects = (),
                to_run = Transition.default()
            )
    

    """ops[f"moveToPos1"] = Operation(
        name = f"moveToPos1", #the name of the operation
        
        precondition=Transition("pre",
            g(f"allowedToMove && !robotAtPos1"),#define a guard for the precondition.
                                                #In the guard above, we say that we only allow the operation to happen if allowedToMove == True && robotAtPos1 == False
            a(f"!allowedToMove")#defines what actions will be taken when the preconditions is evalueted to true, aka what happens when ou begin running the operation
        ),
        
        postcondition=Transition("post",
            g(f"robotAtPos1"), #a guard that checks if we are done with the actions from the precondition
            a(f"allowedToMove, robotPose <- pos1")#actions that will be updated first after the operation has finished. E.g. if you are moving to Pos1 you don't want to update the state robotAtPos1 to True before the robot has finished moving there!
        ),
        # The effects are used to emulate changes of measured variables when planning. For example, when calling the planner,
        # the current state of measured variables might not allow for a plan to be found. Nevertheless, the measured variables 
        # can change during the execution of the plan, so that change is emulated here to trick the planner.
        effects = (),
        to_run = Transition.default()
    )
    
    for name in ["2", "3"]:
        ops[f"moveToPos{name}"] = Operation(
            name = f"moveToPos{name}",
            
            precondition=Transition("pre",
                g(f"allowedToMove && !robotAtPos{name}"),
                a(f"!allowedToMove")
            ),
            
            postcondition=Transition("post",
                g(f"robotAtPos{name}"),
                a(f"allowedToMove, robotPose <- pos{name}")
            ),
            
            to_run = Transition.default()
        )"""
    
    """ops(f"changeVel") = Operation(
        name = f"changeVel",
        
        precondition=Transition("pre",
                g(f""),
                a(f"")
            ),
            
            postcondition=Transition("post",
                g(f""),
                a(f"")
            ),
            
            effects = (),
            to_run = Transition.default()
    )"""
    
    
    # To be used to run "free" transitions. 
    # Example: setting a new goal in a specific state
    transitions: List[Transition] = []

    return Model(
        initial_state,
        ops,
        transitions
    )
    
"""def from_goal_to_goal(state: State) -> Guard:
    """
    #Create a goal predicate 
"""
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()"""
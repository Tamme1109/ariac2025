from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse

@dataclass
class Model(object):
    #state and operations
    initial_state: State #we want to have a state
    operations: Dict[str, Operation] #we want a dictionary with the operations ({nameOfOperation: operation})
    transitions: List[Transition] #we need a list of all the transitions

    def __post_init__(self):
        ops = {o: "i" for o in self.operations}
        self.initial_state = self.initial_state.next(**ops)

g = predicates.guards.from_str #allows us to define guards
a = predicates.actions.from_str#allows us to define what actions we want to happen

def theModel() -> Model:
    initial_state = State(
        #add all the state variables you want here, for simplistic reasons I'll only add a few.
                
        AGV1_allowedToMoveLocation = True,
        AGV1_pos = 'Pos1',
        moveAGV1 = 'init',
        AGV1_moveto = -1,
    )
    
    ops = {} # we store all of our operations in a dict
    
    for i, place in enumerate(['Warehouse', 'Pos1',]):
            ops[f"moveAGV1To{place}"] = Operation(
                name = f"moveAGV1To{place}", #the name of the operation
                precondition=Transition("pre",
                    g(f"AGV1_allowedToMoveLocation && AGV1_pos != {place}"),                                          
                    a(f"!AGV1_allowedToMoveLocation, AGV1_moveto <- {i}")
                ),
                postcondition=Transition("post",
                    g(f"AGV1_pos == {i}"), 
                    a(f"AGV1_allowedToMoveLocation, AGV1_moveto <- {-1}")#AGVPose <- warehouse
                )         
            )
    
    """for nr in range(1,5):
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
                )         
            )"""
    
    
    # To be used to run "free" transitions. 
    # Example: setting a new goal in a specific state
    transitions: List[Transition] = []

    return Model(
        initial_state,
        ops,
        transitions
    )

def from_goal_to_goal(state: State) -> Guard:
    """
    Create a goal predicate 
    """
    goal: str = state.get("goal_as_string")
    if goal != "":
        return g(goal)
    
    return AlwaysFalse()
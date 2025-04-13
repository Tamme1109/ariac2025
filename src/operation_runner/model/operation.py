from __future__ import annotations
from dataclasses import dataclass
from typing import ClassVar, List, Optional, Tuple
import predicates.guards
from predicates.state import State
from predicates.guards import Eq, Guard, guards
from predicates.actions import Action, Assign
from predicates.errors import NextException
from predicates.guards import AlwaysTrue


@dataclass(frozen=True, order=True)
class Transition(object):
    name: str
    guard: Guard
    actions: Tuple[Action, ...]
    """
    A Transition class includes a name of the transition, a guard predicate 
    and a list of actions.
    This transition also include a next_planning that can be used by the operation while planning
    since it does not raise an exception when it is not enabled. This is used to run the postcondition
    in the operation without the need for the guard to be true.
    """

    def eval(self, state: State) -> bool:
        return self.guard.eval(state)

    def next(self, state: State) -> State:
        if not self.eval(state):
            raise NextException(f"Calling next on transition {self.name}, when eval is false")
        
        s = state
        for a in self.actions:
            s = a.next(s)
        return s

    def next_planning(self, state: State) -> State:
        s = state
        for a in self.actions:
            s = a.next(s)
        return s

    @classmethod
    def default(cls) -> Transition:
        return Transition("default",
            AlwaysTrue(),
            ()
        )
        
@dataclass(frozen=True, order=True)
class Operation(object):
    name: str
    precondition: Transition
    postcondition: Transition
    
    def eval(self, state: State) -> bool:
        """Check if the operation can start"""
        init = not state.contains(self.name) or Eq(self.name, "i").eval(state)
        return init and self.precondition.eval(state)

    #we want to use start!
    def start(self, state: State) -> State:
        """Start the operation when running"""
        if not state.contains(self.name):
            state = state.next(**{self.name: "i"})#i means initial
        a = Assign(self.name, "e")#e means executing
        return a.next(self.precondition.next(state))

    #this is basically can be completed (it is checking the current state etc) TODO RENAME TO CAN BE COMPleted
    def is_completed(self, state: State) -> bool:
        """check if the operation has completed while running"""
        return Eq(self.name, "e").eval(state) and self.postcondition.eval(state)


    def complete(self, state: State) -> State:
        """apply the post actions when running"""
        return self.postcondition.next(Assign(self.name, "i").next(state))

        


from dataclasses import dataclass
import json
from typing import List, Optional, Dict
from model.operation import Operation, Transition
from predicates.state import State
import predicates.guards
import predicates.actions
from predicates.guards import AlwaysTrue, Guard, And
from predicates.guards import AlwaysFalse
from model.model import Model, from_goal_to_goal

g = predicates.guards.from_str
a = predicates.actions.from_str


def the_model() -> Model:

    initial_state = State(
        # control variables
        r1_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r1_robot_command = 'move_j',
        r1_robot_velocity = 2.0,
        r1_robot_acceleration = 0.5,
        r1_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r1_robot_tcp_frame = 'r1_svt_tcp', # the tool tcp to use
        r1_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r1_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        r2_robot_run = False,   # trigger action when true. Change to false and then to true to trigger again
        r2_robot_command = 'move_j',
        r2_robot_velocity = 2.0,
        r2_robot_acceleration = 0.5,
        r2_robot_goal_frame = 'unknown',   # where to go with the tool tcp
        r2_robot_tcp_frame = 'r2_svt_tcp', # the tool tcp to use
        r2_gripper_run = False, # trigger service when true. Change to false and then to true to trigger again
        r2_gripper_command = 'none', # pick_red, pick_green, pick_blue, drop_red, drop_green, drop_blue

        #mine------------------------
        r1_above_position = False,
        r2_above_position = False,
        r1_gripper_closed = False,
        r2_gripper_closed = False,

        r1_closest_abovePosition = "above_r1_buffer",
        r2_closest_abovePosition = "above_r2_buffer",

        #---------------------------
        goal_as_string = "",
        replan = False,

        # measured variables
        r1_robot_state = "initial",  # "exec", "done", "failed" 
        r1_robot_pose = "unknown",
        r2_robot_state = "initial",  # "exec", "done", "failed" 
        r2_robot_pose = "unknown",
        replanned = False,

        #estimated
        green_cube_at = "pose_1", # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        red_cube_at = "pose_2",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
        blue_cube_at = "pose_3",  # pose_1, pose_2, pose_3, r1_gripper, r1_buffer, r2_gripper, r2_buffer
    )

    ops = {}
    #move to above positions 1-3
    for i in ["pose_1", "pose_2","pose_3"]:
        for j in ["r1", "r2"]:    
            ops[f"{j}_move_to_above_{i}"] = Operation(
                name = f"{j}_move_to_above_{i}",
                precondition = Transition("pre", 
                                        g(f"{j}_robot_run == False && {j}_robot_state == initial && r1_robot_pose != above_{i} && r2_robot_pose != above_{i} && {j}_closest_abovePosition == above_{i}"), 
                                        a(f"{j}_robot_command = move_j, {j}_robot_run, {j}_robot_goal_frame = above_{i}")),

                postcondition = Transition("post", g(f"{j}_robot_state == done"), 
                                            a(f"!{j}_robot_run, {j}_robot_pose <- above_{i}, {j}_above_position")),

                effects = (),
                to_run = Transition.default()
            )
    
    #move to above individual buffers (this and next operation)
    ops[f"r1_move_to_above_buffer"] = Operation(
                name = f"r1_move_to_above_buffer",
                precondition = Transition("pre", 
                                        g(f"r1_robot_run == False  && r1_robot_state == initial && r1_robot_pose != above_r1_buffer && r1_closest_abovePosition == above_r1_buffer"), #&& r1_robot_pose == r1_buffer
                                        a(f"r1_robot_command = move_j, r1_robot_run, r1_robot_goal_frame = above_r1_buffer")),

                postcondition = Transition("post", g(f"r1_robot_state == done"), 
                                            a(f"!r1_robot_run, r1_robot_pose <- above_r1_buffer, r1_above_position")),

                effects = (),
                to_run = Transition.default()
            )
    ops[f"r2_move_to_above_buffer"] = Operation(
                name = f"r2_move_to_above_buffer",
                precondition = Transition("pre", 
                                        g(f"r2_robot_run == False  && r2_robot_state == initial && r2_robot_pose != above_r2_buffer && r2_closest_abovePosition == above_r2_buffer"), #r2_robot_pose == r2_buffer
                                        a(f"r2_robot_command = move_j, r2_robot_run, r2_robot_goal_frame = above_r2_buffer")),

                postcondition = Transition("post", g(f"r2_robot_state == done"), 
                                            a(f"!r2_robot_run, r2_robot_pose <- above_r2_buffer, r2_above_position")),

                effects = (),
                to_run = Transition.default()
            )

    #move to positions 1-3
    """for i in ["pose_1", "pose_2","pose_3"]:
        for j in ["r1", "r2"]:    
            ops[f"{j}_move_to_{i}"] = Operation(
                name = f"{j}_move_to_{i}",
                precondition = Transition("pre", 
                                        g(f"{j}_robot_run == False && {j}_robot_state == initial && r1_robot_pose != {i} && r2_robot_pose != {i}"), #&& {j}_robot_pose == {i}
                                        a(f"{j}_robot_command = move_j, {j}_robot_run, {j}_robot_goal_frame = {i}")),

                postcondition = Transition("post", g(f"{j}_robot_state == done"), 
                                            a(f"!{j}_robot_run, {j}_robot_pose <- {i}, {j}_closest_abovePosition <- above_{i}")),

                effects = (),
                to_run = Transition.default()
            )"""
    #r1 to pos 1-3
    for i in ["pose_1", "pose_2","pose_3"]:   
        ops[f"r1_move_to_{i}"] = Operation(
            name = f"r1_move_to_{i}",
            precondition = Transition("pre", 
                                    g(f"r1_robot_run == False && r1_robot_state == initial && r1_robot_pose != {i} && r2_robot_pose != {i} && r2_robot_pose != above_{i} && r2_robot_pose != above_pose_2 && r2_robot_pose != pose_2"), #&& {j}_robot_pose == {i}
                                    a(f"r1_robot_command = move_j, r1_robot_run, r1_robot_goal_frame = {i}")),

            postcondition = Transition("post", g(f"r1_robot_state == done"), 
                                        a(f"!r1_robot_run, r1_robot_pose <- {i}, r1_closest_abovePosition <- above_{i}")),

            effects = (),
            to_run = Transition.default()
        )
    #r2 to pos 1-3
    for i in ["pose_1", "pose_2","pose_3"]:   
        ops[f"r2_move_to_{i}"] = Operation(
            name = f"r2_move_to_{i}",
            precondition = Transition("pre", 
                                    g(f"r2_robot_run == False && r2_robot_state == initial && r2_robot_pose != {i} && r1_robot_pose != {i} && r1_robot_pose != above_{i} && r1_robot_pose != above_pose_2 && r1_robot_pose != pose_2"), #&& {j}_robot_pose == {i}
                                    a(f"r2_robot_command = move_j, r2_robot_run, r2_robot_goal_frame = {i}")),

            postcondition = Transition("post", g(f"r2_robot_state == done"), 
                                        a(f"!r2_robot_run, r2_robot_pose <- {i}, r2_closest_abovePosition <- above_{i}")),

            effects = (),
            to_run = Transition.default()
        )
    #move to individual buffers (this and the next operation)
    ops[f"r1_move_to_buffer"] = Operation(
                name = f"r1_move_to_buffer",
                precondition = Transition("pre", 
                                        g(f"r1_robot_run == False  && r1_robot_state == initial && r1_robot_pose != r1_buffer"), 
                                        a(f"r1_robot_command = move_j, r1_robot_run, r1_robot_goal_frame = r1_buffer")),

                postcondition = Transition("post", g(f"r1_robot_state == done"), 
                                            a(f"!r1_robot_run, r1_robot_pose <- r1_buffer, r1_closest_abovePosition <- above_r1_buffer")),

                effects = (),
                to_run = Transition.default()
            )
    ops[f"r2_move_to_buffer"] = Operation(
                name = f"r2_move_to_buffer",
                precondition = Transition("pre", 
                                        g(f"r2_robot_run == False  && r2_robot_state == initial && r2_robot_pose != r2_buffer"), 
                                        a(f"r2_robot_command = move_j, r2_robot_run, r2_robot_goal_frame = r2_buffer")),

                postcondition = Transition("post", g(f"r2_robot_state == done"), 
                                            a(f"!r2_robot_run, r2_robot_pose <- r2_buffer, r2_closest_abovePosition <- above_r2_buffer")),

                effects = (),
                to_run = Transition.default()
            )

    """ #pick from positions 1-3
    for i in ["red", "green", "blue"]:
        for j in ["pose_1", "pose_2","pose_3"]:
            for k in ["r1", "r2"]:
                ops[f"{k}_pick_{i}_at_{j}"] = Operation(
                    name = f"{k}_pick_{i}_at_{j}",

                    precondition = Transition("pre",
                                              g(f"{k}_above_position == True && {i}_cube_at == {j} && !{k}_gripper_run && {k}_robot_pose == {j} && !{k}_robot_run && !{k}_gripper_closed"),
                                              a(f"{k}_gripper_command = pick_{i}, {k}_gripper_run, {k}_robot_run, {k}_robot_command = move_j, {k}_gripper_closed")),
                    postcondition = Transition("post",
                                               g(f"{k}_robot_state == done"),
                                               a(f"!{k}_gripper_run, {i}_cube_at <- {k}_gripper, !{k}_robot_run, !{k}_above_position ")), #robot_pose <- above_{j}
                    effects = (),
                    to_run = Transition.default()
                )"""
  
    #pick up for r1
    for i in ["red", "green", "blue"]:
        for j in ["r1_buffer", "pose_1", "pose_2","pose_3"]:
            ops[f"r1_pick_{i}_at_{j}"] = Operation(
                name = f"r1_pick_{i}_at_{j}",

                precondition = Transition("pre",
                                          g(f"r1_above_position == True && {i}_cube_at == {j} && !r1_gripper_run && r1_robot_pose == {j} && !r1_robot_run && !r1_gripper_closed"),
                                          a(f"r1_gripper_command = pick_{i}, r1_gripper_run, r1_robot_run, r1_robot_command = move_j, r1_gripper_closed")),
                postcondition = Transition("post",
                                           g(f"r1_robot_state == done"),
                                           a(f"!r1_gripper_run, {i}_cube_at <- r1_gripper, !r1_robot_run, !r1_above_position ")), #robot_pose <- above_{j}
                effects = (),
                to_run = Transition.default()
            )
    #pick up for r2
    for i in ["red", "green", "blue"]:
        for j in ["r2_buffer", "pose_1", "pose_2","pose_3"]:
            ops[f"r2_pick_{i}_at_{j}"] = Operation(
                name = f"r2_pick_{i}_at_{j}",

                precondition = Transition("pre",
                                          g(f"r2_above_position == True && {i}_cube_at == {j} && !r2_gripper_run && r2_robot_pose == {j} && !r2_robot_run && !r2_gripper_closed"),
                                          a(f"r2_gripper_command = pick_{i}, r2_gripper_run, r2_robot_run, r2_robot_command = move_j, r2_gripper_closed")),
                postcondition = Transition("post",
                                           g(f"r2_robot_state == done"),
                                           a(f"!r2_gripper_run, {i}_cube_at <- r2_gripper, !r2_robot_run, !r2_above_position")), #robot_pose <- above_{j}
                effects = (),
                to_run = Transition.default()
            )

    """#pick from one of the buffers
    for i in ["red", "green", "blue"]:
        for k in ["r1", "r2"]:
            ops[f"{k}_pick_{i}_at_{j}"] = Operation(
                name = f"{k}_pick_{i}_at_{j}",

                precondition = Transition("pre",
                                            g(f"{k}_above_position == True && {i}_cube_at == {k}_buffer && !{k}_gripper_run && {k}_robot_pose == {k}_buffer && !{k}_robot_run && !{k}_gripper_closed"),
                                            a(f"{k}_gripper_command = pick_{i}, {k}_gripper_run, {k}_robot_run, {k}_robot_command = move_j, {k}_gripper_closed")),
                postcondition = Transition("post",
                                            g(f"{k}_robot_state == done"),
                                            a(f"!{k}_gripper_run, {i}_cube_at <- {k}_gripper, !{k}_robot_run, !{k}_above_position ")), #robot_pose <- above_{j}
                effects = (),
                to_run = Transition.default()
            )"""

    """   #place cubes on positions 1-3
    for i in ["red", "green", "blue"]:
        for j in ["pose_1", "pose_2","pose_3"]:
            for k in ["r1", "r2"]:
                ops[f"{k}_place_{i}_at_{j}"] = Operation(
                    name = f"{k}_place_{i}_at_{j}",

                    precondition = Transition("pre",
                                                g(f"{k}_above_position == True && {i}_cube_at == {k}_gripper && !{k}_gripper_run && {k}_robot_pose == {j} && !{k}_robot_run && {k}_gripper_closed && red_cube_at != {j} && blue_cube_at != {j} && green_cube_at != {j}"),
                                                a(f"{k}_gripper_command <- place_{i}, {k}_gripper_run, {k}_robot_run, {k}_robot_command = move_j, !{k}_gripper_closed")),
                    postcondition = Transition("post",
                                                g(f"{k}_robot_state == done"),
                                                a(f"!{k}_gripper_run, {i}_cube_at <- {j}, {k}_gripper_command <- none, !{k}_robot_run, !{k}_above_position ")), 
                    effects = (),
                    to_run = Transition.default()
                )
    #place cubes on buffers
    for i in ["red", "green", "blue"]:
        for k in ["r1", "r2"]:
            ops[f"{k}_place_{i}_at_buffer"] = Operation(
                name = f"{k}_place_{i}_at_{j}",

                precondition = Transition("pre",
                                            g(f"{k}_above_position == True && {i}_cube_at == {k}_gripper && !{k}_gripper_run && {k}_robot_pose == {k}_buffer && !{k}_robot_run && {k}_gripper_closed && red_cube_at != {k}_buffer && blue_cube_at != {k}_buffer && green_cube_at != {k}_buffer"),
                                            a(f"{k}_gripper_command <- place_{i}, {k}_gripper_run, {k}_robot_run, {k}_robot_command = move_j, !{k}_gripper_closed")),
                postcondition = Transition("post",
                                            g(f"{k}_robot_state == done"),
                                            a(f"!{k}_gripper_run, {i}_cube_at <- {k}_buffer, {k}_gripper_command <- none, !{k}_robot_run, !{k}_above_position ")), 
                effects = (),
                to_run = Transition.default()
            )"""
    #place cube with r1
    for i in ["red", "green", "blue"]:
        for j in ["r1_buffer", "pose_1", "pose_2","pose_3"]:
            ops[f"r1_place_{i}_at_{j}"] = Operation(
                name = f"r1_place_{i}_at_{j}",

                precondition = Transition("pre",
                                          g(f"r1_above_position == True && {i}_cube_at == r1_gripper && !r1_gripper_run && r1_robot_pose == {j} && !r1_robot_run && r1_gripper_closed && red_cube_at != {j} && blue_cube_at != {j} && green_cube_at != {j}"),
                                          a(f"r1_gripper_command <- place_{i}, r1_gripper_run, r1_robot_run, r1_robot_command = move_j, !r1_gripper_closed")),
                postcondition = Transition("post",
                                           g(f"r1_robot_state == done"),
                                           a(f"!r1_gripper_run, {i}_cube_at <- {j}, r1_gripper_command <- none, !r1_robot_run, !r1_above_position")), 
                effects = (),
                to_run = Transition.default()
            )
    #place cube with r2
    for i in ["red", "green", "blue"]:
        for j in ["r2_buffer", "pose_1", "pose_2","pose_3"]:
            ops[f"r2_place_{i}_at_{j}"] = Operation(
                name = f"r2_place_{i}_at_{j}",

                precondition = Transition("pre",
                                          g(f"r2_above_position == True && {i}_cube_at == r2_gripper && !r2_gripper_run && r2_robot_pose == {j} && !r2_robot_run && r2_gripper_closed && red_cube_at != {j} && blue_cube_at != {j} && green_cube_at != {j}"),
                                          a(f"r2_gripper_command <- place_{i}, r2_gripper_run, r2_robot_run, r2_robot_command = move_j, !r2_gripper_closed")),
                postcondition = Transition("post",
                                           g(f"r2_robot_state == done"),
                                           a(f"!r2_gripper_run, {i}_cube_at <- {j}, r2_gripper_command <- none, !r2_robot_run, !r2_above_position")), 
                effects = (),
                to_run = Transition.default()
            )
  

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
import os
from typing import List, Tuple, Optional
import time

import numpy as np

from neem_interface_python.rosprolog_client import Prolog, atom
from neem_interface_python.utils import Datapoint

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class NEEMError(Exception):
    pass


class NEEMInterface:
    """
    Low-level interface to KnowRob, which enables the easy creation of NEEMs in Python.
    For more ease of use, consider using the Episode object in a 'with' statement instead (see below).
    """
    def __init__(self):
        self.prolog = Prolog()

        # Load neem-interface.pl into KnowRob
        neem_interface_path = os.path.join(SCRIPT_DIR, os.pardir, "neem-interface.pl")
        self.prolog.once(f"ensure_loaded({atom(neem_interface_path)})")


    ### NEEM Creation ###############################################################

    def start_episode(self, task_type: str, env_owl: str, env_owl_ind_name: str, env_urdf: str,
                      agent_owl: str, agent_owl_ind_name: str, agent_urdf: str, start_time: float = None):
        """
        Start an episode and return the prolog atom for the corresponding action.
        """
        q = f"mem_episode_start(Action, {atom(task_type)}, {atom(env_owl)}, {atom(env_owl_ind_name)}, {atom(env_urdf)}," \
            f"{atom(agent_owl)}, {atom(agent_owl_ind_name)}, {atom(agent_urdf)}," \
            f"{start_time if start_time is not None else time.time()})"
        res = self.prolog.once(q)
        return res["Action"]

    def stop_episode(self, neem_path: str, end_time: float = None):
        """
        End the current episode and save the NEEM to the given path
        """
        return self.prolog.once(f"mem_episode_stop({atom(neem_path)}, {end_time if end_time is not None else time.time()})")

    def add_subaction_with_task(self, parent_action, sub_action_type="dul:'Action'", task_type="dul:'Task'",
                                start_time: float = None, end_time: float = None) -> str:
        """
        Assert a subaction of a given type, and an associated task of a given type.
        """
        q = f"mem_add_subaction_with_task({atom(parent_action)},{atom(sub_action_type)},{atom(task_type)},SubAction)"
        solution = self.prolog.once(q)
        if solution is not None:
            action_iri = solution["SubAction"]
            if start_time is not None and end_time is not None:
                self.prolog.once(f"kb_project(has_time_interval({atom(action_iri)}, {start_time}, {end_time}))")
            return action_iri
        else:
            raise NEEMError("Failed to create action")

    def assert_tf_trajectory(self, points: List[Datapoint]):
        print(f"Inserting {len(points)} points")
        for point in points:
            ee_pose_str = point.to_knowrob_string()
            self.prolog.once(f"""
                time_scope({point.timestamp}, {point.timestamp}, QS),
                tf_set_pose({atom(point.frame)}, {ee_pose_str}, QS).
            """)

    def assert_transition(self, agent_iri: str, object_iri: str, start_time: float, end_time: float) -> Tuple[str, str, str]:
        res = self.prolog.once(f"""
            kb_project([
                new_iri(InitialScene, soma:'Scene'), is_individual(InitialScene), instance_of(InitialScene, soma:'Scene'),
                new_iri(InitialState, soma:'State'), is_state(InitialState),
                has_participant(InitialState, {atom(object_iri)}),
                has_participant(InitialState, {atom(agent_iri)}),
                holds(InitialScene, dul:'includesEvent', InitialState),
                has_time_interval(InitialState, {start_time}, {start_time}),

                new_iri(TerminalScene, soma:'Scene'), is_individual(TerminalScene), instance_of(TerminalScene, soma:'Scene'),
                new_iri(TerminalState, soma:'State'), is_state(TerminalState),
                has_participant(TerminalState, {atom(object_iri)}),
                has_participant(TerminalState, {atom(agent_iri)}),
                holds(TerminalScene, dul:'includesEvent', TerminalState),
                has_time_interval(TerminalState, {end_time}, {end_time}),

                new_iri(Transition, dul:'Transition'), is_individual(Transition), instance_of(Transition, soma:'StateTransition'),
                holds(Transition, soma:'hasInitialScene', InitialScene),
                holds(Transition, soma:'hasTerminalScene', TerminalScene)
            ]).
        """)
        transition_iri = res["Transition"]
        initial_state_iri = res["InitialState"]
        terminal_state_iri = res["TerminalState"]
        return transition_iri, initial_state_iri, terminal_state_iri

    def assert_agent_with_effector(self, effector_iri: str, agent_type="dul:'PhysicalAgent'", agent_iri: str = None) -> str:
        if agent_iri is None:
            agent_iri = self.prolog.once(f"""
                kb_project([
                    new_iri(Agent, dul:'Agent'), is_individual(Agent), instance_of(Agent, {atom(agent_type)})
                ]).""")["Agent"]
        self.prolog.once(f"has_end_link({atom(agent_iri)}, {atom(effector_iri)})")
        return agent_iri

    def assert_state(self, participant_iris: List[str], start_time: float = None, end_time: float = None, state_type="soma:'State'") -> str:
        state_iri = self.prolog.once(f"""
            kb_project([
                new_iri(State, {atom(state_type)}), is_individual(State), instance_of(State, {atom(state_type)})
            ])
        """)["State"]
        if start_time is not None and end_time is not None:
            self.prolog.once(f"kb_project(has_time_interval({atom(state_iri)}, {start_time}, {end_time}))")
        for iri in participant_iris:
            self.prolog.once(f"kb_project(has_participant({atom(state_iri)}, {atom(iri)}))")
        return state_iri

    def assert_situation(self, agent_iri: str, involved_objects: List[str], situation_type="dul:'Situation'") -> str:
        situation_iri = self.prolog.once(f"""
            kb_project([
                new_iri(Situation, {atom(situation_type)}), is_individual(Situation), instance_of(Situation, {atom(situation_type)}),
                holds(Situation, dul:'includesAgent', {atom(agent_iri)})
            ])
        """)["Situation"]
        for obj_iri in involved_objects:
            self.prolog.once(f"kb_project(holds({atom(situation_iri)}, dul:'includesObject', {atom(obj_iri)}))")
        return situation_iri

    ### NEEM Parsing ###############################################################

    def load_neem(self, neem_path: str):
        """
        Load a NEEM into the KnowRob knowledge base.
        """
        self.prolog.once(f"remember({atom(neem_path)})")

    def get_all_actions(self) -> List[str]:
        res = self.prolog.all_solutions("is_action(Action)")
        if len(res) > 0:
            return list(set([dic["Action"] for dic in
                             res]))  # Deduplicate: is_action(A) may yield the same action more than once
        else:
            raise NEEMError("Failed to find any actions")

    def get_interval_for_action(self, action: str) -> Optional[Tuple[float, float]]:
        res = self.prolog.once(f"event_interval({atom(action)}, Begin, End)")
        if res is None:
            return res
        return res["Begin"], res["End"]

    def get_tf_trajectory(self, obj: str, start_timestamp: float, end_timestamp: float) -> List:
        res = self.prolog.once(f"tf_mng_trajectory({atom(obj)}, {start_timestamp}, {end_timestamp}, Trajectory)")
        return res["Trajectory"]

    def get_wrench_trajectory(self, obj: str, start_timestamp: float, end_timestamp: float) -> List:
        res = self.prolog.once(f"wrench_mng_trajectory({atom(obj)}, {start_timestamp}, {end_timestamp}, Trajectory)")
        return res["Trajectory"]


class Episode:
    """
    Convenience object and context manager for NEEM creation. Can be used in a 'with' statement to automatically
    start and end a NEEM context (episode).
    """
    def __init__(self, neem_interface: NEEMInterface, task_type: str, env_owl: str, env_owl_ind_name: str,
                 env_urdf: str,
                 env_urdf_prefix: str, agent_owl: str, agent_owl_ind_name: str, agent_urdf: str, neem_output_path: str,
                 start_time=None):
        self.neem_interface = neem_interface
        self.task_type = task_type
        self.env_owl = env_owl
        self.env_owl_ind_name = env_owl_ind_name
        self.env_urdf = env_urdf
        self.env_urdf_prefix = env_urdf_prefix
        self.agent_owl = agent_owl
        self.agent_owl_ind_name = agent_owl_ind_name
        self.agent_urdf = agent_urdf
        self.neem_output_path = neem_output_path

        self.top_level_action_iri = None
        self.episode_iri = None
        self.start_time = start_time if start_time is not None else time.time()

    def __enter__(self):
        self.top_level_action_iri = self.neem_interface.start_episode(self.task_type, self.env_owl,
                                                                      self.env_owl_ind_name, self.env_urdf, self.agent_owl,
                                                                      self.agent_owl_ind_name, self.agent_urdf,
                                                                      self.start_time)
        self.episode_iri = \
            self.neem_interface.prolog.once(f"kb_call(is_setting_for(Episode, {atom(self.top_level_action_iri)}))")[
                "Episode"]
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.neem_interface.stop_episode(self.neem_output_path)

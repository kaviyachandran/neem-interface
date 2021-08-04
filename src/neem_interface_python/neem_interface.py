import os
from typing import List, Tuple

from neem_interface_python.rosprolog_client import Prolog, atom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class NEEMError(Exception):
    pass


class NEEMInterface:
    def __init__(self):
        self.prolog = Prolog()

        # Load neem-interface.pl into KnowRob
        neem_interface_path = os.path.join(SCRIPT_DIR, os.pardir, "neem-interface.pl")
        self.prolog.once(f"ensure_loaded({atom(neem_interface_path)})")

        self.current_episode = None

    ### NEEM Creation ###############################################################

    def start_episode(self, task_type: str, env_owl: str, env_owl_ind_name: str, env_urdf: str, env_urdf_prefix: str,
                      agent_owl: str, agent_owl_ind_name: str, agent_urdf: str):
        """
        Start an episode and return the prolog atom for the corresponding action.
        """
        q = f"mem_episode_start(Action, {atom(task_type)}, {atom(env_owl)}, {atom(env_owl_ind_name)}, {atom(env_urdf)}," \
            f"{atom(env_urdf_prefix)}, {atom(agent_owl)}, {atom(agent_owl_ind_name)}, {atom(agent_urdf)})"
        res = self.prolog.once(q)
        return res["Action"]

    def stop_episode(self, neem_path: str):
        return self.prolog.once(f"mem_episode_stop('{neem_path}')")

    def add_subaction_with_task(self, parent_action, sub_action_type, task_type) -> str:
        """
        """
        q = f"add_subaction_with_task({atom(parent_action)},{atom(sub_action_type)},{atom(task_type)},SubAction)"
        solution = self.prolog.once(q)
        if solution is not None:
            return solution["SubAction"]
        else:
            raise NEEMError("Failed to create action")


class Episode:
    def __init__(self, neem_interface: NEEMInterface, task_type: str, env_owl: str, env_owl_ind_name: str,
                 env_urdf: str,
                 env_urdf_prefix: str, agent_owl: str, agent_owl_ind_name: str, agent_urdf: str, neem_output_path: str):
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

    def __enter__(self):
        self.top_level_action_iri = self.neem_interface.start_episode(self.task_type, self.env_owl,
                                                                      self.env_owl_ind_name, self.env_urdf,
                                                                      self.env_urdf_prefix, self.agent_owl,
                                                                      self.agent_owl_ind_name, self.agent_urdf)
        self.episode_iri = \
            self.neem_interface.prolog.once(f"kb_call(is_setting_for(Episode, {atom(self.top_level_action_iri)}))")[
                "Episode"]
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.neem_interface.stop_episode(self.neem_output_path)

"""Launch node specifications replicated per vehicle namespace.

Centralizes the ``/{vehicle}/{sub_ns}/{node_name}`` naming convention that the
parameter override machinery (node selectors, regex section expansion, CLI
overrides) matches against.
"""

from typing import Dict, Iterable, List, Optional

from avt_341_param_lib.runtime.launch_params import perform_yaml


class NodeSpec:
    """Static launch specification for one node replicated per vehicle."""

    def __init__(self, executable, template=None, condition=None, sub_ns=None,
                 extra_params=None, output='screen', disallowed_vehicles_arg=None):
        self.executable = executable
        # one template path or a list of paths (a node whose executable links
        # several generated parameter services); normalized to a list
        self.templates = [template] if isinstance(template, str) else list(template or [])
        self.condition = condition          # callable(context) -> bool; None = always
        self.sub_ns = sub_ns                # sub-namespace below the vehicle namespace
        self.extra_params = extra_params    # callable(vid, vehicles) -> dict of launch-computed params
        self.output = output
        # name of a launch argument holding a yaml list of the vehicle ids the
        # node must NOT spawn on, performed at spawn time; None = unrestricted
        self.disallowed_vehicles_arg = disallowed_vehicles_arg

    def allows_vehicle(self, context, vid) -> bool:
        """Whether the node may spawn on the vehicle (``disallowed_vehicles_arg`` gate)."""
        if not self.disallowed_vehicles_arg:
            return True
        value = perform_yaml(context, self.disallowed_vehicles_arg)
        if value is None:
            value = []
        if not isinstance(value, list):
            raise RuntimeError(
                f"Launch argument '{self.disallowed_vehicles_arg}' must hold a "
                f"list of vehicle ids, got {value!r}")
        return str(vid).strip('/') not in {str(v).strip('/') for v in value}


def node_fqn(name: str, vid, sub_ns: Optional[str] = None) -> str:
    """Fully qualified name of a node instance under a vehicle namespace."""
    node_name = str(name).rsplit('/', 1)[-1]
    namespace_parts = [str(vid).strip('/')]
    if sub_ns:
        namespace_parts.extend(str(sub_ns).strip('/').split('/'))
    return '/' + '/'.join([*namespace_parts, node_name])


def candidate_node_fqns(node_specs: Dict[str, NodeSpec], vehicles: Iterable) -> List[str]:
    """FQNs of every candidate node of every vehicle, ignoring spawn gating.

    ``condition`` and ``disallowed_vehicles_arg`` are deliberately not
    evaluated: the candidate set feeds selector matching, where entries for
    un-spawned nodes are harmless dead yaml.
    """
    return [
        node_fqn(name, vid, spec.sub_ns)
        for vid in vehicles
        for name, spec in node_specs.items()
    ]

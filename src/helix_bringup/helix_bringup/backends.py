"""Backend selection for the HELIX nodes that have a native C++ port.

Python stays the default for every component until the C++ build is
validated on the robot. Each C++ package installs its executable under the
same node name as the Python reference, with the same parameters, topics and
lifecycle, so a backend choice changes only which package is launched.

The launch files call these helpers from an OpaqueFunction, so an invalid or
contradictory choice stops the launch with a clear message instead of
silently starting the wrong node or two motion publishers.
"""

BACKENDS = ('python', 'cpp')

# component -> backend -> (package, executable)
EXECUTABLES = {
    'anomaly_detector': {
        'python': ('helix_core', 'helix_anomaly_detector'),
        'cpp': ('helix_sensing_cpp', 'helix_anomaly_detector'),
    },
    'arbiter': {
        'python': ('helix_arbiter', 'helix_arbiter'),
        'cpp': ('helix_arbiter_cpp', 'helix_arbiter'),
    },
}

_TRUE = ('true', '1', 'yes', 'on')
_FALSE = ('false', '0', 'no', 'off')


def parse_bool(name: str, value: str) -> bool:
    """Parse a launch-argument boolean the way launch's IfCondition does."""
    text = value.strip().lower()
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    raise ValueError(f'{name} must be true or false, got {value!r}')


def resolve_backend(name: str, value: str) -> str:
    """Return a validated backend name for the launch argument ``name``."""
    choice = value.strip().lower()
    if choice not in BACKENDS:
        raise ValueError(f'{name} must be one of {list(BACKENDS)}, got {value!r}')
    return choice


def resolve_anomaly_backend(anomaly_backend: str, use_cpp_anomaly: str) -> str:
    """Resolve the anomaly detector backend from the new and legacy arguments.

    ``anomaly_backend`` is the explicit choice. Empty means "not set", in
    which case the legacy ``use_cpp_anomaly`` flag decides. Setting both to
    values that disagree is an error rather than a hidden precedence rule.
    """
    legacy_cpp = parse_bool('use_cpp_anomaly', use_cpp_anomaly)
    if anomaly_backend.strip() == '':
        return 'cpp' if legacy_cpp else 'python'
    choice = resolve_backend('anomaly_backend', anomaly_backend)
    if legacy_cpp and choice != 'cpp':
        raise ValueError(
            'use_cpp_anomaly:=true conflicts with anomaly_backend:=python; '
            'set only anomaly_backend')
    return choice


def resolve_arbiter_backend(arbiter_backend: str, enable_twist_mux: str):
    """Return the arbiter backend, or None when legacy twist_mux replaces it.

    twist_mux and the arbiter are mutually exclusive owners of the velocity
    output, so asking for the C++ arbiter together with twist_mux is refused.
    """
    choice = resolve_backend('arbiter_backend', arbiter_backend)
    if parse_bool('enable_twist_mux', enable_twist_mux):
        if choice != 'python':
            raise ValueError(
                'enable_twist_mux:=true replaces the arbiter; '
                f'arbiter_backend:={choice} would be ignored, so it is refused')
        return None
    return choice


def executable(component: str, backend: str):
    """Return (package, executable) for ``component`` on ``backend``."""
    return EXECUTABLES[component][backend]

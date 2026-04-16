"""Shared helper: publish a labeled float metric on /helix/metrics."""
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


def make_metric(name: str, value: float) -> Float64MultiArray:
    msg = Float64MultiArray()
    dim = MultiArrayDimension()
    dim.label = name
    dim.size = 1
    dim.stride = 1
    msg.layout = MultiArrayLayout()
    msg.layout.dim = [dim]
    msg.data = [value]
    return msg

"""helix_arbiter tests get their own DDS domain (see helix_bringup/conftest)."""
import os

os.environ['ROS_DOMAIN_ID'] = '87'

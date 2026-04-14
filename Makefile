# HELIX development targets
.PHONY: test test-sim build clean

# Disable pytest plugin autoload to avoid the broken anyio/pytest-asyncio combo
# at /home/yusuf/.local/lib/python3.10/site-packages/anyio/pytest_plugin.py
# which fails with `No module named '_pytest.scope'` on pytest 6.2.5.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD := 1

build:
	colcon build --cmake-clean-cache

test:
	colcon test --pytest-args -v
	colcon test-result --verbose

test-sim:
	@echo "REQUIREMENT: Isaac Sim must be running with the go2_ros2_bridge (see docs/)"
	python3 -m pytest tests/sim_integration/ -v -s

clean:
	rm -rf build install log

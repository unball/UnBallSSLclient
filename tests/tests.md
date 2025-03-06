# Run all unit tests
python -m unittest discover tests/unit

# Test individual components
python -m tests.unit.test_path_follower
python -m tests.unit.test_robot_states
python -m tests.unit.test_state_machine

# Run all integration tests
python -m unittest discover tests/integration

# Test robot behavior with path planning
python -m tests.integration.test_behavior

# Test specific scenarios
python -m tests.integration.test_scenarios

# Run visual field test
python -m tests.visual.test_visualization

# Test with specific division
python -m tests.visual.test_visualization --division "Entry Level"

# Test path visualization
python -m tests.visual.test_path_visualization

# Run full system integration test
python -m tests.system.test_integration

# Command-line test utility with various options
python -m tests.system.test_cli --role goalkeeper --duration 20 --verbose
python -m tests.system.test_cli --role defender --duration 15
python -m tests.system.test_cli --role attacker --duration 30
python -m tests.system.test_cli --role all --duration 60

# Test with mock vision data
python -m tests.system.test_cli --mock

# Run all tests
python -m unittest discover tests

# Run test suite with detailed output
python -m tests.run_all_tests
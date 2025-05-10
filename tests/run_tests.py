#!/usr/bin/env python
"""
Test runner for UAV Strategic Deconfliction System.

This script discovers and runs all tests in the tests directory.
"""
import unittest
import os
import sys

# Add the parent directory to system path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

def run_tests():
    """Discover and run all tests."""
    # Discover tests in the current directory
    test_loader = unittest.TestLoader()
    test_suite = test_loader.discover(os.path.dirname(__file__), pattern='test_*.py')
    
    # Run the tests
    test_runner = unittest.TextTestRunner(verbosity=2)
    result = test_runner.run(test_suite)
    
    # Return success or failure
    return result.wasSuccessful()

if __name__ == '__main__':
    print("Running UAV Strategic Deconfliction System tests...")
    success = run_tests()
    
    # Set exit code based on test success
    sys.exit(0 if success else 1) 
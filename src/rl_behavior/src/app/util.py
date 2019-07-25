"""Definition of utility functions.
"""

# ------------------------------------------------------------------------------
# CODE ATTRIBUTION NOTICE
# This code is closely based off of the example implementation for the Python
# 3.5 math.isclose() function, as noted in PEP-485
def isclose(first, second, rel_tol=1e-9, abs_tol=0.0):
  return abs(first-second) <= max(
    rel_tol * max(abs(first), abs(second)), abs_tol
  )
# END CODE ATTRIBUTION NOTICE
# ------------------------------------------------------------------------------
# vim: set ts=2 sw=2 expandtab:

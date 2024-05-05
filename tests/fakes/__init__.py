"""Test doubles that behave like the real thing badly, on purpose.

- :mod:`mechanics` models the rotator: gear reduction, backlash, an endstop at
  a physical angle, hard stops beyond it, and a motor that loses steps when it
  is asked for more than it can deliver.
- :mod:`fake_lgpio` models the Pi's GPIO library, including the contract
  violations a real one would reject.
"""

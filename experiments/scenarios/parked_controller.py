"""Keep an opponent stationary in local multi-car evidence scenarios.

The controller returns zero steering and speed so a teleported vehicle can act
as a repeatable obstacle while the primary controller is evaluated. It is not
included in either submission build.
"""


def control(left_img, right_img, timestamp):
    return 0.0, 0.0

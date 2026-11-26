"""Perception: what the robot sees, and what it makes of it.

Three pieces, deliberately separable:

* ``camera``   - renders the scene from the robot's head.
* ``detector`` - turns a frame into ``Detection`` records. Ground truth from
                 MuJoCo today, a learned detector later, behind one interface.
* ``overlay``  - draws detections and verdicts onto a frame.
* ``capture``  - writes stills, a contact sheet and a video of a round.

Nothing in here decides an inspection verdict. The checklist in
``shared/inspection.py`` reads the simulation directly and remains the only
thing that judges a station, so navigation and reporting do not depend on
perception working. That is the governing constraint from the brief, and it is
what makes this package safe to add late.
"""

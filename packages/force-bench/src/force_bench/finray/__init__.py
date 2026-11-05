"""Fin Ray soft fingers on the FR3: geometry, adapter frames, simulation models, the ``finray`` embodiment.

``geometry``    the measured finger profile (walls, ribs) and the adapter frames (assets/finray)
``frame_fe``    a linear 2-D frame (beam) model of the profile
``frame_nl``    the same walls, geometrically exact, pressed by a cylinder: the stiffness reference
``prb``         the pseudo-rigid-body (PRB) linkage; ``prb_cpu`` solves it quasi-statically
``model``       Newton models of the finger on the FR3 carriages: rigid or PRB
``embodiment``  ``FinRayFingers``, the hooks a task calls (``force-bench run <task> --embodiment finray``)
``deform``      how much the simulated fingers bend, from recorded body poses (the deformation gate)
``probe``       the cylinder press in the simulator; ``report`` draws it against the references
See docs/finray.md and docs/embodiments.md.
"""

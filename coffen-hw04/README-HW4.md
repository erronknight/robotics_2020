Sarah Coffen
https://www.youtube.com/watch?v=JebrShBVL_o&feature=youtu.be
The planning strategy is a greedy kind of planning strategy. It iterates
over the moon list for a given body and selects the next "target" as the
one that is closest in inclination and semimajoraxis to the ship's orbit
at that time. Then it proceeds to do a target maneuver node, followed by
circularization and inclination change as needed. I did this since it allows
for more intermediate changes (the shifting between target elliptical orbits
to circular orbits), and thus easier to scale, despite me not scaling it.
Used exec_node from http://ksp-kos.github.io/KOS_DOC/tutorials/exenode.html?highlight=node and
used http://www.braeunig.us/space/orbmech.htm for math help.
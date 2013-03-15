README
======

There are currently 4 swing functions that work reasonably well:

- swingSimpleCycloid: The simplest case for which you create a cycloid between (x0,y0) and (x1,y1) (joined by a straight line in xy)
- swingEllipse: (I see this works well! Xinyan fill info here)
- swing2Cycloids: 2 Cycloids, one for z and other for xy velocities. Produces continuous profiles for x,y,z. Still joined with straight line
- swingSimpleBezier: Bezier for xy plane (no straight line as all the others). Cycloid for z

Comparative plots of different results with these functions are in the folder named plots

Note: Since I understand that this function will be merged in Step 2 I am not making a library out of it :)

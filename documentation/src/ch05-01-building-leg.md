# Building The Legs
In this chapter we will be building the 4 legs.
## Required 3D-printed parts per leg:
- Left legs:
	- [Leg-Top-Left](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Top-Left.stl)
	- [Leg-Bottom-Left](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Bottom-Left.stl)
	- [Leg-Joint-Top-Left](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Joint-Top-Left.stl)
	- [Leg-Joint-Bottom-Left](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Joint-Bottom-Left.stl)
	- [Sock](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Sock.stl) (Print out of TPU, this part is to provide grip)
- Right legs:
	-  [Leg-Top-Right](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Top-Right.stl)
	- [Leg-Bottom-Right](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Bottom-Right.stl)
	- [Leg-Joint-Top-Right](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Joint-Top-Right.stl)
	- [Leg-Joint-Bottom-Right](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Leg-Joint-Bottom-Right.stl)
	- [Sock](https://github.com/DeDiamondPro/AQLARP/blob/master/models/stl/legs/Sock.stl) (Print out of TPU, this part is to provide grip)
## Assembly
First, take the top part of the leg and attach the servo in the hole, the gear of the servo should be facing the same way as the hole at the top for the circular servo horn. Then attach the circular servo horn in the top hole using 4 small screws. After doing those 2 steps, it should look something like this.
![](img/Legs-img1.png)
Then attach the `Leg-Joint-Top` and `Leg-Joint-Bottom` part together using an M3 screw and a locknut. Don't overtighten it, it should be able to rotate freely. Then attach it to the bottom leg part, after doing that it should look something like this.
![](img/Legs-img2.png)
Then connect the top and bottom of the leg together using an M4 screw and a locknut, again don't overtighten it since it should be able to rotate freely. After doing that you should have something like this.
![](img/Legs-img3.png)
Then finally you must set the servo to 90 degrees, to do this connect the servo to the servo driver and run the following command on the Raspberry Pi.
```console
$ python3 ~/AQLARP/scripts/set90.py
```
Then connect the top joint part to the servo, try to make the angle as close to 90° as possible. Then lock them together using an M3 screw.

And that is a completed leg, the rest of the servos will be added when assembling the body since if you attach them now you won't be able to access some parts that you will need to access later.
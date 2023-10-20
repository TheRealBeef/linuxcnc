```
This linuxcnc fork has a cmake project under the hood.
To build linuxcnc with cmake, visit ~/linuxcnc/cmake/ then execute the script : ./build_all

To open and edit linuxcnc source code in a designer like qt, open the ~/linuxcnc/cmake/cmakelists.txt
After building linuxcnc inside qt, don't forget to do a "make install". 
```

```
In the ~/linuxcnc/component/motdot i have added code for scurve jogging.
This jogging already works and can be reviewed. 
A ini value for jerk_max has to be added to the source code.
See the file simple_tp.c for more information.

```

```
In the ~/linuxcnc/component/homemod i have added code to home the machine without moving motors.
There is a hal pin : joint.home_all, set this pin to high, then press the home all button.
The machine is then homed on all axis without moving any machine motor. 

```
```
before a github push, perform the ~linuxcnc/cmake/./clean_all command.

```

[![Badge GPL2]][License]
[![Badge LGPL]][License]

<div align = center>

<br>
  
# LinuxCNC
  
*Controlling CNC Machines*

<br>
  
[![Badge Translation]][Translation]
  
<br>
  
---

[<kbd> <br> Ｗｅｂｓｉｔｅ <br> </kbd>][Website] 
[<kbd> <br> Ｉｎｓｔａｌｌ <br> </kbd>][Ｉｎｓｔａｌｌ] 
[<kbd> <br> Ｂｕｉｌｄ <br> </kbd>][Ｂｕｉｌｄ] 
[<kbd> <br> Ｄｏｃｕｍｅｎｔａｔｉｏｎ <br> </kbd>][Ｄｏｃｕｍｅｎｔａｔｉｏｎ]  
  
---

<br>
  
It can drive milling machines, lathes, 3D printers, laser <br>
cutters, plasma cutters, robot arms, hexapods, and more.

LinuxCNC was initiated 25 years ago and evolved into a very <br>
international project with contributions from all over the globe.
  
With release 2.9 of LinuxCNC we also transitioned the <br>
documentation to the use of the public crowd translation <br>
services [Weblate] and invite all our users to contribute.
  
The translations we expect to help attract practitioners <br>
to the project and also helps educating enthusiasts of <br>
all age groups on automated machining.

<br>

## DISCLAIMER
  
<br>

```
  
Ｔｈｅ ａｕｔｈｏｒｓ ｏｆ ｔｈｉｓ ｓｏｆｔｗａｒｅ ａｃｃｅｐｔ
ａｂｓｏｌｕｔｅｌｙ ｎｏ ｌｉａｂｉｌｉｔｙ ｆｏｒ ａｎｙ
ｈａｒｍ　ｏｒ ｌｏｓｓ ｒｅｓｕｌｔｉｎｇ ｆｒｏｍ ｉｔｓ ｕｓｅ．

Ｉｔ ｉｓ ＥＸＴＲＥＭＥＬＹ ｕｎｗｉｓｅ ｔｏ　ｒｅｌｙ
ｏｎ ｓｏｆｔｗａｒｅ ａｌｏｎｅ ｆｏｒ ｓａｆｅｔｙ．

Any machinery capable of harming persons must have
provisions for completely removing power from all
motors, etc., before persons enter any danger area.

All machinery must be designed to comply with local 
and national safety codes, and the authors of this 
software cannot and do not, take any responsibility 
for such compliance.
  
```

<br>
  
</div>

<!----------------------------------------------------------------------------->

[Badge Translation]: https://hosted.weblate.org/widgets/linuxcnc/-/svg-badge.svg
[Badge GPL2]: https://img.shields.io/badge/Most-LGPL_3-blue.svg?style=for-the-badge 'The license this software is under'
[Badge LGPL]: https://img.shields.io/badge/Some-GPL_2-blue.svg?style=for-the-badge 'Some parts are under this license'

[Translation]: https://hosted.weblate.org/engage/linuxcnc/
[Weblate]: https://hosted.weblate.org/projects/linuxcnc/
[Website]: https://linuxcnc.org/

[Ｄｏｃｕｍｅｎｔａｔｉｏｎ]: http://linuxcnc.org/docs/devel/html/
[Ｉｎｓｔａｌｌ]: http://linuxcnc.org/docs/devel/html/getting-started/getting-linuxcnc.html
[Ｂｕｉｌｄ]: http://linuxcnc.org/docs/devel/html/code/building-linuxcnc.html
[License]: COPYING

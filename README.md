# ScueDK
Embedded communication &amp; data access library for Scue.

## ROS
### Download & Build
```
cd ~/catkin_ws/src
git clone https://github.com/jinkim31/ScueDK.git
cm
```
### Include
1. Edit CMakeList.
```
#Add "scuedk" in find_package. Following is an example!
find_package(catkin REQUIRED qt_build roscpp scuedk)
```
2. Include library.
```
#include "schedk.h"
```

3. Make Scue instance.
```
//In  QNode.h
Scue<scue::Master> *scue; //Master is the master struct defined in structs.h.

//In QNode.cpp 
scue = new Scue<scue::Master>(n); //n is nodehandle
```

# ScueDK
Embedded communication &amp; data access library for Scue.

## ROS
### Build
```
cd ~/catkin_ws/src
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/jinkim31/ScueDK.git scuedk
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

## Embedded master
### Clone source
```
cd <your project dir (not Core!)>
git clone https://github.com/jinkim31/ScueDK.git scuedk
```
### Include
```
#include "../../ScueDK/scuedk_embedded/master/MasterSerialLine.h"
#include "../../ScueDK/structs/structs.h"
using namespace scue;
```


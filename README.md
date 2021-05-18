**BiDirectional RRT**

## Description
*C++ module that explores a given environment area creating rapidly exploring random trees expanding towards 
each other and then determining a path to take while avoiding obstacles. This algorithm
is used for a 2d environment. For determining the route, the robot will be treated as holonomic.*

---

Random Object placement with no Optimizations example 1: <br />

![random_notOptimized](https://user-images.githubusercontent.com/58401719/118682571-93750d80-b7ce-11eb-9340-40504224e28d.gif)

User Defined Object placement: <br />

![userDefined_notOptimized](https://user-images.githubusercontent.com/58401719/118682626-9f60cf80-b7ce-11eb-8024-e11e98c52946.gif)

User Defined Object placement with path Optimizations: <br />

![userDefined_optimized](https://user-images.githubusercontent.com/58401719/118682641-a25bc000-b7ce-11eb-964c-9fcfb9b50416.gif)

Random Object placement with path Optimizations: <br />

![random_optimized](https://user-images.githubusercontent.com/58401719/118682603-9b34b200-b7ce-11eb-852c-54941eb9fc71.gif)

## Cloning the repo

git clone https://github.com/JakeInit/RRT.git

---

## Dependencies
Ensure all dependencies below are installed: <br />  <br />
sudo apt-get update <br />
sudo apt-get install lsb-release -y <br />
sudo apt-get install -y \ <br />
&nbsp;&nbsp;clang \ <br />
&nbsp;&nbsp;g++ \ <br />
&nbsp;&nbsp;doxygen \ <br />
&nbsp;&nbsp;git \ <br />
&nbsp;&nbsp;build-essential \ <br />
&nbsp;&nbsp;linux-libc-dev \ <br />
&nbsp;&nbsp;cmake \ <br />
&nbsp;&nbsp;cmake-qt-gui \ <br />
&nbsp;&nbsp;libusb-1.0-0-dev \ <br />
&nbsp;&nbsp;libusb-dev \ <br />
&nbsp;&nbsp;libudev-dev \ <br />
&nbsp;&nbsp;libgtest-dev \ <br />
&nbsp;&nbsp;freeglut3-dev \ <br />
&nbsp;&nbsp;gcc \ <br />
&nbsp;&nbsp;graphviz

**json**<br />
sudo apt-get install libjsoncpp-dev

**boost**<br />
sudo apt-get install libboost-all-dev

**eigen3**<br />
sudo apt-get install libeigen3-dev

**octomap**<br />
sudo apt-get install liboctomap-dev

**libccd**<br />
git clone https://github.com/danfis/libccd.git <br />
cd libccd <br />
mkdir build && cd build <br />
cmake -G "Unix Makefiles" .. <br />
make <br />
make install <br />

**fcl**<br />
git clone https://github.com/flexible-collision-library/fcl.git <br />
cd fcl <br />
mkdir build && cd build <br />
cmake .. <br />
make <br />
make install <br />

or on ubuntu <br />
sudo apt-get install libfcl-dev

**SFML**<br />
sudo apt-get install libopenal-dev <br />
sudo apt-get install -y vorbis-tools <br />
sudo apt-get install libvorbis-dev <br />
sudo apt install libflac-dev <br />

git clone https://github.com/SFML/SFML.git <br />
cd SFML <br />
mkdir build && cd build <br />
cmake .. <br />
make <br />
make install <br />

---

## Building and Running the Project
	mkdir build && cd build
	cmake ..
	make
	cd ..
	./bin/rrtApplication
	
	note: make can be run from the main repo directory. This will run cmake and make. <br />
	Then just run ./bin/rrtApplication
	
---

## Configurations Parameters Found in Json
1. loop_frequency_Hz:    determines how fast the loop time of the application is run at <br />
2. maxNodes:             the max number of attempts to connect the trees <br />
3. visualizerHeight_pix: visualizer window height in pixels, <br />
4. visualizerWidth_pix:  visualizer window width in pixels <br />
5. stepDistance_m:       max distance to expand the tree on each iteration <br />
6. minStepDistance_m:    min distance to expand the tree on goal iteration <br />
7. boundaryHeight_m:     the +/- max y values of the coordinate system <br />
8. boundaryWidth_m:      the +/- max x values of the coordinate system <br />
9. maxObjects:           max number of objects to be randomly generated in the map <br />
10. minObjects:           min number of objects to be randomly generated in the map <br />
12. maxObjectSize_m:      square objects are generated form 0.100m to a max size of this value <br />
13. pathSmootherOn:       attempts to smooth the path taken if enabled <br />
14. UserDefinedObjectsOn  Places user defined objects in graph instead of random placement <br />
15. width_m:              robot width <br />
16. height_m:             robot height <br />
17. buffer_m:             robot buffer for gap to leave betwen objects

The below is an example of user defined objects. The value "UserDefinedObjectsON" must be <br />
set to true to use these. Otherwise random objects are created. Objects are defined by 2D <br />
x, y coordinates. Any new object and any number can be added using the format below. <br />
<br />
IMPORTANT:
"User defined objects must be axis aligned for both x and y. Shapes must be square or rectangle. <br />
<br />
"object_1": [ <br />
&nbsp;&nbsp;[-2, -2], <br />
&nbsp;&nbsp;[-2, 2], <br />
&nbsp;&nbsp;[2, -2], <br />
&nbsp;&nbsp;[2, 2] <br />
], <br />
"object_2": [ <br />
&nbsp;&nbsp;[-3, -3], <br />
&nbsp;&nbsp;[-2.5, -2.5], <br />
&nbsp;&nbsp;[-2.5, -3], <br />
&nbsp;&nbsp;[-3, -2.5] <br />
] <br />

---

## Notes:
1. User configurations exist in pathToRepo/RRT/json/config.json
2. Objects in visualizer will be white squares as well as borders
3. The robot in visualizer will be a blue square
4. The goal point in visualizer will be a red circle
5. The RRT in visualizer will made up of red line segments
6. The path taken in visualizer will be outlined in green
7. List of poses of path taken is written to pathToRepo/RRT/poses/poses.txt
8. Too many objects can make path impossible and impossible to place robot and goal point
9. Too large of objects can make path impossible and impossible to place robot and goal point

---


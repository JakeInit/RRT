**Binary RRT**

## Description
*C++ module that explores a given environment area creating trees expanding towards 
each other and then determining a path to take while avoiding obstacles. This algorithm
is used for a 2d environment. The robot is considered differential driven with wheel centered.
Therefore the robot can be treated as holonomic since it can turn in place to align with
the next goal point and drive straight.*

---

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
	
---

## Notes:
1. User configurations exist in pathToRepo/RRT/json/config.json
2. Robot height and width should be the same
3. Boundary height and width should be the same
4. Visualizer height and width should be the same
5. Objects in visualizer will be white squares
6. The robot in visualizer will be a blue square
7. The goal point in visualizer will be a red circle
8. The RRT in visualizer will made up of red line segments
9. The path taken in visualizer will be outlined in green

---


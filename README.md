**Binary RRT**

*C++ module that explores a given environment area creating trees expanding towards 
each other and then determining a path to take while avoiding obstacles*

## Cloning the repo

git clone https://github.com/JakeInit/RRT.git

---

## Dependencies

**eigen3**<br />
sudo apt-get install libeigen3-dev

**Octomap**<br />
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

---

## Building and Running the Project
	mkdir build && cd build
	cmake ..
	make
	cd ..
	./bin/rrtApplication
	
---


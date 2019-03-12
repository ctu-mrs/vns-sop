## [Variable Neighborhood Search for the Set Orienteering Problem and its application to other Orienteering Problem variants](https://doi.org/10.1016/j.ejor.2019.01.047)

VNS-SOP repository provide source code and benchmark datasets of the [Variable Neighborhood Search method for the Set Orienteering Problem](https://doi.org/10.1016/j.ejor.2019.01.047), published in the European Journal of Operational Research. Please contact authors of the [paper](https://doi.org/10.1016/j.ejor.2019.01.047) regarding problems with running bellow described code. 

**Repository of contains following content:**
- **datasets** folder contains datasets of SOP
- **results** folder contains computational results shown in the [paper](https://doi.org/10.1016/j.ejor.2019.01.047)
- **sources** folder contains source codes for VNS and ILP for SOP  


### Dependencies 

For Ubuntu 18.04 LTS the dependencies can be installed using apt as:
```bash 
sudo apt-get install make ccache build-essential pkg-config liblog4cxx-dev libcairo2-dev libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev libboost-iostreams-dev libboost-system-dev
```
### Cloning and compilation of supporting libraries
To clone the repository do:
```bash 
git clone --recursive git@github.com:ctu-mrs/vns-sop.git
```
and compile supporting library using:
```bash 
cd vns-sop/sources/comrob/crl/
./install.sh
```


### Compilation

The VNS-SOP can be compiled using the above dependencies by running _make sop_vns_ in folder _vns-sop/sources_. 
The ILP additionally requires CPLEX dependency where the ILP formulation for SOP is implemented. Please set the CPLEX_ROOT_DIR variable in Makefile to right location of your CPLEX installation. The ILP for SOP can be compiled by running _make sop_ilp_. Alternatively both VNS-SOP and ILP for SOP can be compiled at once by calling _make all_.

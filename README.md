## [Variable Neighborhood Search for the Set Orienteering Problem and its application to other Orienteering Problem variants](https://doi.org/10.1016/j.ejor.2019.01.047)

VNS-SOP repository provide source code and benchmark datasets of the [Variable Neighborhood Search method for the Set Orienteering Problem](https://doi.org/10.1016/j.ejor.2019.01.047), published in the European Journal of Operational Research. Please contact authors of the [paper](https://doi.org/10.1016/j.ejor.2019.01.047) regarding problems with running bellow described code. 

**Repository contains following content:**
- **datasets** folder contains dataset instances of SOP (including SOP for DOP and OPN)
- **results** folder contains computational results shown in the [paper](https://doi.org/10.1016/j.ejor.2019.01.047)
- **sources** folder contains source codes for VNS and ILP solvers of SOP 
- **visualization** folder contains python scripts for visualizing the computed results 


### Dependencies 

For Ubuntu 18.04 LTS the dependencies can be installed using apt as:
```bash 
sudo apt-get install make ccache build-essential pkg-config liblog4cxx-dev libcairo2-dev libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev libboost-iostreams-dev libboost-system-dev
```
### Cloning and compilation of supporting libraries
To clone the repository run
```bash 
git clone --recursive https://github.com/ctu-mrs/vns-sop.git
```
and compile supporting library in vns-sop/sources/comrob/crl/ folder by running
```bash 
cd vns-sop/sources/comrob/crl/
./install.sh
```


### Compilation

The VNS-SOP can be compiled using the above dependencies by running **make sop\_vns** in folder **vns-sop/sources**. 
The ILP additionally requires CPLEX solver in which the ILP formulation of SOP is implemented. Please set the CPLEX_ROOT_DIR variable in vns-sop/sources/Makefile to right location of your CPLEX installation. The ILP for SOP can be then compiled by running **make sop\_ilp**. Alternatively, both VNS-SOP and ILP for SOP can be compiled at once by calling **make all**.

### Running VNS-SOP and ILP for SOP

After compilation, the VNS-SOP can be run using **sop\_vns** program and the ILP solver for SOP can be run using **sop\_ilp** program.
The default configuration of programs is stored in **sop.cfg** file. 
The most important configuration parameters are:
- _problem_ - specifies location of sop dataset instance file
- _gui_ - switch between "cairo" gui, "none" gui and "cairo-nowin" with background creation of gui images (gui is only shown for pure SOP and OPN instances, the DOP instances have no gui in runtime, however, can be visualized using the show_solution.py script)
- _nowait_ - switch whether to close gui window after finish of the solver

The configuration parameters can be also set as a command line parameters, e.g. by running
```bash
./sop_vns --problem=../datasets/opn_sop_dataset/tsiligirides_problem_2_budget_30_d_50_s_08.sop --gui=cairo --nowait=0
```

### Visualization

Visualization script **show_solution.py** in visualization folder can be used to show the last solution recorded in result log **sources/results/results.log**.
To be able to run the script, following dependencies have to be installed first (for the Ubuntu 18.04 LTS):
```bash
sudo apt-get install python3-numpy python3-matplotlib python3-scipy python3-shapely python3-descartes python3-pip
pip3 install git+git://github.com/AndrewWalker/pydubins.git
```
Then, the visualization script can be run by calling **./show_solution.py** showing the matplotlib graph of latest result and saving it to png image. Variant of SOP (pure SOP, DOP or OPN) is determined based on problem file location.

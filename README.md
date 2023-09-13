# CL-ECBS and CL-PIBT

## Overview

In this repo, we provide the implementation of CL-ECBS, CL-CBS, CL-PIBT, SHA* algorithms.



The video demonstration can be found here

[![IMAGE ALT TEXT](image.png)](https://youtu.be/ua31edNfau0?si=APlJP0wSuRbjC1TV)

## Source Code
### Requirement
```bash
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
clang-tidy clang-format python3-matplotlib libompl-dev libeigen3-dev
```
> Note: Please make sure your `matplotlib` version is above `2.0`, otherwise it may show weird image while visualization. You can upgrade it by `pip3 install -U matplotlib`.


### Build
```bash
cd static #or cd lifelong
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release  ..
make -j8
```

### Instance Generation
Users can use "generator.py" to generate random instances. All instances will be saved as json files.

### Run instances
```bash
./CL-ECBS -i input.json -o output.json
./PIBT -i input.json -o output.json
```

### Visualize Results
```bash
# make sure your are in build folder
python3 ../src/visualize.py -m  input.json -s output.json
```
### Acknowledgement
We would like to express our gratitude to Licheng Wen and their team for generously sharing their code.

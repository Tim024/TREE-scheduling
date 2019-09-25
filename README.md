# contiki-ng-TREE

This repository contains the implementation of the TREE scheduling algorithm.

The algorithm is implemented in *os/services/TREE*. An example is provided in *examples/TREE/TREE-rpl*. The folders contains the border-router code (*br.c*) and the node code (*node.c*). Hyper-parameters are in *project-conf.h*. To build an example with tree, the makefile must contain the following lines:
```
MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/TREE
```

# How to experiment on FiT/IoT-LAB

- Clone this repository on your local machine.
- Navigate to your application code, or use the example in *examples/TREE/TREE-rpl/*
- Make for the M3 target with `make TARGET=iotlab BOARD=m3` which should give you a compiled border router file *br.iotlab* and node file *node.iotlab*.
- Note that for the M3 target to correctly work with TSCH, the hyper parameters must contain the lines
```
#define RF2XX_WITH_TSCH 1
#define RF2XX_SOFT_PREPARE 0
```
- Login to the FIT/IoT-LAB testbed and schedule M3 devices for experiments.
- Upload the border router code to one of the device, and the node code the other devices.
- The communication output can be vizualized on the FIT/IoT-LAB website.

Further information about the FIT/IoT-LAB can be found [here](https://www.iot-lab.info/tutorials/contiki-ng-compilation/).

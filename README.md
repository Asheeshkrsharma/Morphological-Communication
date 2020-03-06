# Instructions

The project is divided in three parts

1. Robota: Contains code for the simulation
2. Spring: Contains the code for the underlying elastica (2D spring) simulation.
3. Valley: Contains the code for the underlying flow-field simulation and terrain generation.

*Note You can find some notes on how individual parts were implemented in the docs directory*

## Directory tree

.
├── docs
│   ├── results.pdf
│   └── Terrain generation and roughness.pdf
├── Robota
│   └── Robota.pde
│   └── scheduler.py
├── spring
│   └── spring.pde
└── Valley
    └── Valley.pde

# Dependencies

Processing => 3.2 [processing.org](processing.org)

Box2D https://github.com/shiffman/Box2D-for-Processing

ControlP5 http://www.sojamo.de/libraries/controlp5

giCentre utilities https://www.gicentre.net/utils

You can find more about installing these libraries at [processing.org](processing.org)

# To control the processing sketch from python
Please look at scheduler.py


# Citation

Note: You are not yet allowed to release or redistribute this code since, we are in a process of publishing a paper which uses this code. For your dissertation:

If you use this code, please cite us as follows.

```latex
Sharma, A (as16542@my.bristol.ac.uk), Hauser, H (helmut.hauser@bristol.ac.uk) and Hauert, S (sabine.hauert@bristol.ac.uk) 2019. Morphologically Communicating Swarms. 
```

In the case that you need to share this or any of its modified form with anyone, please inform Helmut, Sabine and me first.

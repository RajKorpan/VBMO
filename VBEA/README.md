# Preface
We use the following naming convention
- _J_ for the number of objectives in the graph.
- _K_ be the size of the population, and
- _T_ be the number of number of generations.

### VBEA Parameters
#### __Hyper Parameters:__
| | |
|---- | ---- |  
| ___K___ | The size of the child population (default _J_)  | 
| ___T___ | The number of generations (default 0)
|___voting_method___ |  voting method used for evaluation. See ??? for list of supported methods |
| ___h___ | The heuristic function. See  ???? for a list of supported heuristic functions |
|___child_method___ | child population method. See ??? for list of supported methods |
|

- ___voting_method___  currently supports:
    - _range_
    - _condorcet_
    - _combined_approval_
    - _borda_

- __h__ currently supports 
    - _euclidean_
    - _haversine_

- __child_method__ currently supports
    - weighted conscious combined
    - weighted combined

2. __graph environment:__
-  ASCII
    additional parameters for what objectives
-  DOT
    ???

3. Optional Params
- instances file 
- Visualization 
    True or False (will effect performance)


### Creating Instances

```
<map name 1>
i <source> <target>
i <source> <target>
...
<map name 2>
i <source> target> 
...
```

### Creating Files

#### ASCII/Grid

```
<name>
height < height >  
width < width:w>  

<width x height ASCII map>
```

where the map consist of either 
 - @ = empty space
 - . = node
 - T = Wall 

example
```
myfunnymap
height 15
width 52

..................@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTT
..................................TTTTT...........TTT
.................TTTTTTTTTTTTTTTT......TTTT......TTTT
.........TTTTT...TTTTTTTTT.........TT...TTT.......T..
.........TTTTTTTTTTTTTTTTT...........T...............
...............TTTTTTTTTTT...........TT..............
.....................TTTTT...........TT..............
.....................................TT..............
......................................T..............
.....................................TT..............
.....................................TT..............
.....................TTTTT..........TTT..............
....................TTTTTT.TTTTTTTTTTTT..............
..............TTTTTTTTTTTTTTTTTTTTTTTT...............
.........TTTTTTTTTTTTTTTTTTTTTTTTTTTT...TTT..........
.........TT......TTTTTTTTTTTTTTTTTTT...TTTT......TTTT
.................@@@@@@@@TTTTTTTTTTTTTTTTTTTTTTTTTTTT
```

#### Graph 

Require separate graphical and objective files 

- Graph:

```
<HEADER>
<HEADER>
<HEADER>

c <node count> 
c 
c <node id> <white space separated information > 
c <node id> <white space separated information > 
c ...
c <node id> <white space separated information > 
```

e.g.

```
FunnyGraph.txt
Created by the Giant Rat That Makes All of Da Rules
using global coordinates, 

c 5
c 
c 1 -124132 12312
c 2 -124152 13346
c 3 -141455 12456
c 4 -215123 1241
c 5 -23151 12512
```

The objective files will also need to be provided in the format 

```
<HEADER>
<HEADER>
<HEADER>

c <edge count>
c <source> <target> <cost>
...
c <source> <target <cost>>
```

e.g. 

```
FunnyGraph.txt
Distance File
Created by John Pork

c 15
c 1 2 100
c 2 3 15
c 1 4 45
...
c 4 5 76
```


### Question:
> What happens when the K < number of graph objectives for gen0?
     
 Picks at random the the single objective or creates a set of random weight sets?  
          
> What happens when K > number of of graph objectives

Using the normalized scores of the first K single objective path to create weight sets  & || totally at random weight sets.  

> What happens if there are not enough weight sets to fill the population

use either randomly created weight, or some quick method of using previous weight sets to create new ones.
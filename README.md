#  D*_lite_grip_planning
This is a simple grid based implementation for D*lite algorithm, which is proposed by Seven Koenig.
D*lite is a performance-wise dynamic re-planning algorithm for path planning in a unknown environment, which is based on
life-long planning A*. It proposed a new parameter, rhs, for each node and constantly checking if the current rhs equals the minimum values of the g(s') + c(s,s'). When the environment changed, the condition no longer hold and it will recompute the path based on the updated environment.



## Build and run
```
git clone https://github.com/zzjkf2009/Dstar_lite_grip_planning
mkdir build
cd build
cmake ..
make
```
To run the code
```
./app/shell-app
```
to run the unit test
```
./test/cpp-test
```

## Result
![gif-1](https://github.com/zzjkf2009/Dstar_lite_grip_planning/blob/master/result/1.gif)
![gif-2](https://github.com/zzjkf2009/Dstar_lite_grip_planning/blob/master/result/2.gif)

# Drone Delivery Simulation
To use the simulation:
1. Clone the repo
2. Build project
3. Run ./app executable with input file
```
./app < drone-test-large1000.txt
```

# Input File Format
```
<n drones>
<m delivery locations>
<0 or 1 to print centroids>
<m new line separated coordinate pairs>
    eg. 15 -32
    x, y ∈ [-COORD_LIMIT, COORD_LIMIT]
```

# Example Output
```
Drone 1 cost: 2728.36
Drone 2 cost: 1794.21
Drone 3 cost: 2405.91
Drone 4 cost: 3417.81
Drone 5 cost: 2592.07
Drone 6 cost: 2600.31
Drone 7 cost: 2473.58
Drone 8 cost: 2283.28
Drone 9 cost: 2392.19
Drone 10 cost: 2525.64
```
![droneRoutes](https://github.com/ShivGovil/droneDelivery/assets/26761109/708be9cf-adfb-4a43-9281-ba82c5c90981)

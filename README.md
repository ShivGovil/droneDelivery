# Drone Delivery Simulation
To use the simulation:
1. Clone the repo
2. Build project
3. Run app executable with input file redirection
```
./app < drone-test-large.txt
```

# Example Output
```
Drone 1 cost: 186.93
Drone 2 cost: 192.40
Drone 3 cost: 146.28
Drone 4 cost: 213.88
Drone 5 cost: 175.93
Drone 6 cost: 176.53
Drone 7 cost: 216.98
Drone 8 cost: 202.62
Drone 9 cost: 154.05
Drone 10 cost: 186.89
```
<img width="998" alt="Screenshot 2023-12-30 at 4 20 25 PM" src="https://github.com/ShivGovil/droneDelivery/assets/26761109/dd4a0aa4-2d3a-4696-afac-db3edbff58e7">

# Input File Format
```
<n drones>
<m delivery locations>
<m new line separated coordinate pairs>
    eg. 15 -32
    x, y belong to [-100, 100]
```

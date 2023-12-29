#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <vector>

#include <GLFW/glfw3.h>
using namespace std;

struct DropZone {
    double x;
    double y;
    bool visited;
    double distance;
    size_t centroid;
};

class DroneFlight {
public:
    vector<DropZone> drops;
    size_t dropsCount = 0;
    vector<size_t> bestPath;
    double bestPathCost = 0;

    double findMST(size_t permLength) {
        for (size_t i = permLength; i < dropsCount; i++) {
            drops[currentPath[i]].visited = false;
            drops[currentPath[i]].distance = numeric_limits<double>::infinity();
        }

        drops[currentPath[permLength]].distance = 0;

        size_t currentNode = 0;

        for (size_t loop = permLength; loop < dropsCount; loop++) {
            double minDistance = numeric_limits<double>::infinity();
            for (size_t i = permLength; i < dropsCount; i++) {
                if (!drops[currentPath[i]].visited && drops[currentPath[i]].distance < minDistance) {
                    minDistance = drops[currentPath[i]].distance;
                    currentNode = currentPath[i];
                }
            }

            drops[currentNode].visited = true;

            for (size_t i = permLength; i < dropsCount; i++) {
                if (drops[currentPath[i]].visited) continue;

                double distanceFromCurrentToOtherConnected = distNoSqrt(drops[currentNode], drops[currentPath[i]]);
                
                if (drops[currentPath[i]].distance > distanceFromCurrentToOtherConnected)
                    drops[currentPath[i]].distance = distanceFromCurrentToOtherConnected;

            }
        }

        double cost = 0;
        for (size_t i = permLength; i < dropsCount; i++) {
            cost += sqrt(drops[currentPath[i]].distance);
        }

        return cost;
    }

    void arbitraryTSP() {
        bestPath.push_back(0);
        bestPath.push_back(0);

        double totalCost = 0;

        for (size_t i = 1; i < drops.size(); i++) {
            size_t minInsertionIndex = 0;
            double minCost = numeric_limits<double>::infinity();

            for (size_t j = 0; j < bestPath.size() - 1; j++) {
                double newCost = dist(drops[bestPath[j]], drops[i])
                + dist(drops[i], drops[bestPath[j+1]])
                - dist(drops[bestPath[j]], drops[bestPath[j+1]]);

                if (newCost < minCost) {
                    minCost = newCost;
                    minInsertionIndex = j;
                }
            }

            totalCost += minCost;

            bestPath.insert(bestPath.begin() + ((int)minInsertionIndex + 1), i);
        }

        bestPathCost = totalCost;

        bestPath.pop_back();

        reverse(bestPath.begin() + 1, bestPath.end());

        currentPath = bestPath;
    }

    void genPerms(size_t permLength) {
        if (permLength == dropsCount) {
            double d = dist(drops[currentPath[0]], drops[currentPath[dropsCount - 1]]);
            currentPathCost += d;
            if (bestPathCost >= currentPathCost) {
                bestPathCost = currentPathCost;
                bestPath = currentPath;
            }
            currentPathCost -= d;
            return;
        }  // if ..complete path

        if (!promising(permLength)) {
            return;
        }  // if ..not promising

        for (size_t i = permLength; i < dropsCount; i++) {
            swap(currentPath[permLength], currentPath[i]);
            currentPathCost += dist(drops[currentPath[permLength]], drops[currentPath[permLength - 1]]);;
            genPerms(permLength + 1);
            currentPathCost -= dist(drops[currentPath[permLength]], drops[currentPath[permLength - 1]]);;
            swap(currentPath[permLength], currentPath[i]);
        }  // for ..unpermuted elementsa
    }  // genPerms()

private:
    vector<size_t> currentPath;
    double currentPathCost = 0;

    bool promising(size_t permLength) {
        if (dropsCount - permLength <= 4) return true;

        double costOfMST = findMST(permLength);

        double minCostFirst = numeric_limits<double>::infinity();
        double minCostLast = numeric_limits<double>::infinity();
        for (size_t i = permLength; i < dropsCount; i++) {
            double costFirst = distNoSqrt(drops[currentPath[i]], drops[currentPath[0]]);
            if (costFirst < minCostFirst) {
                minCostFirst = costFirst;
            }
            
            double costLast = distNoSqrt(drops[currentPath[i]], drops[currentPath[permLength - 1]]);
            if (costLast < minCostLast) {
                minCostLast = costLast;
            }
        }

        if (currentPathCost + costOfMST + sqrt(minCostFirst) + sqrt(minCostLast) < bestPathCost) {
            return true;
        }

        return false;
    }

    double dist(DropZone& p1, DropZone& p2) {
        return sqrt(((p2.x - p1.x)*(p2.x - p1.x)) + ((p2.y - p1.y)*(p2.y - p1.y)));
    }

    double distNoSqrt(DropZone& p1, DropZone& p2) {
        return ((p2.x - p1.x)*(p2.x - p1.x)) + ((p2.y - p1.y)*(p2.y - p1.y));
    }

};

void kMeans(vector<DropZone>& locations) {

}

int main(int argc, char* argv[]) {
    ios_base::sync_with_stdio(false);
    // cout << setprecision(2);
    // cout << fixed;

    size_t droneCount;

    cin >> droneCount;

    const size_t MIN_DRONE_COUNT = 1;
    const size_t MAX_DRONE_COUNT = 5;

    if (droneCount < MIN_DRONE_COUNT || droneCount > MAX_DRONE_COUNT) {
        cerr << "Drone fleet size must be between 1 and 5. Exiting...\n";
        exit(1);
    }

    vector<DroneFlight> drones(droneCount);

    for (size_t drone = 0; drone < droneCount; drone++) {
        drones[drone] = DroneFlight();
    }

    size_t num;
    cin >> num;

    vector<DropZone> locations(num);

    size_t curr = 0;
    size_t centroidNum = 0;

    for (size_t i = 0; i < num; i++) {
        cin >> locations[i].x;
        if (locations[i].x < -100 || locations[i].x > 100) {
            cerr << "Coordinates must be between -100 and 100. Exiting...\n";
            exit(1);
        }

        cin >> locations[i].y;
        if (locations[i].y < -100 || locations[i].y > 100) {
            cerr << "Coordinates must be between -100 and 100. Exiting...\n";
            exit(1);
        }
        
        locations[i].distance = numeric_limits<double>::infinity();
        locations[i].visited = false;
        if (curr > num / droneCount) {
            curr = 0;
            centroidNum++;
        }
        locations[i].centroid = centroidNum;
        curr++;
    }

    // kMeans(locations);

    for (auto& location : locations) {
        drones[location.centroid].dropsCount++;
        drones[location.centroid].drops.push_back(location);
    }

    size_t count = 1;
    for (auto& drone : drones) {
        drone.arbitraryTSP();
        drone.genPerms(1);
        cout << "Drone " << count++ << " cost: " << drone.bestPathCost << "\n";
    }

    // --------------------GFLW BEGIN-----------------------

    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1000, 1000, "Hello World", NULL, NULL);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    const vector<vector<double>> colors = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1},
        {1, 1, 0},
        {0, 1, 1}
    };

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);
        
        glPointSize(20);
        glLineWidth(3);
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_COLOR_MATERIAL);

        size_t count = 0;
        
        for (auto& drone : drones) {
            glBegin(GL_POINTS);
            glColor3f(colors[count][0], colors[count][1], colors[count][2]);
            count++;
            for (size_t i = 0; i < drone.dropsCount; i++) {
                glVertex2f(drone.drops[drone.bestPath[i]].x/100, drone.drops[drone.bestPath[i]].y/100);
            }
            glEnd();
        }

        count = 0;

        for (auto& drone : drones) {
            glBegin(GL_LINE_LOOP);
            glColor3f(colors[count][0], colors[count][1], colors[count][2]);
            count++;
            for (size_t i = 0; i < drone.dropsCount; i++) {
                glVertex2f(drone.drops[drone.bestPath[i]].x/100, drone.drops[drone.bestPath[i]].y/100);
            }
            glEnd();
        }

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();
    }

    glfwTerminate();

    // --------------------GFLW END-----------------------

    return 0;
}
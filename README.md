# **Enhanced RRT-Connect Path Planning for Mobile Robots**

## **Project Overview**
This repository contains the implementation and comparison of **path planning algorithms** for mobile robots, inspired by the paper:  
📝 **"Improved RRT-Connect Based Path Planning Algorithm for Mobile Robots."**  

The repository includes implementations of:
- **RRT (Rapidly Exploring Random Tree)**
- **RRT* (Optimized RRT)**
- **RRT-Connect (Bidirectional RRT)**
- **Improved RRT-Connect** (Midpoint-enhanced bidirectional RRT)

The **Improved RRT-Connect algorithm** introduces a third **midpoint node**, which reduces search time and optimizes path convergence by structuring the tree into four regions.

---

## **Key Features**
✅ **Efficient Path Planning**: Uses **sampling-based algorithms** to navigate in complex environments.  
✅ **Optimized Search Strategy**: The improved RRT-Connect algorithm integrates **midpoint biasing**, improving path convergence speed.  
✅ **Bidirectional Tree Growth**: Expands trees from both start and goal positions, reducing computational complexity.  
✅ **Obstacle Avoidance**: Avoids predefined obstacles using a **clearance-aware approach**.  
✅ **Performance Comparison**: Evaluates RRT, RRT*, RRT-Connect, and Improved RRT-Connect on **various start-goal configurations**.  

---

## **Project Structure**
```plaintext
improved_rrt_connect/
│── rrt.py                  # Basic RRT implementation
│── rrt_star.py             # Optimized RRT* algorithm
│── rrt_connect.py          # Standard RRT-Connect algorithm
│── improves_rrt_connect.py # Improved RRT-Connect with midpoint strategy
│── annotated_report.pdf    # Detailed research paper with results
│── requirements.txt        # List of dependencies
│── README.md               # Documentation for the project
│── results/
│   ├── output_rrt.mp4      # Visualization of RRT paths
│   ├── output_rrt_star.mp4 # Visualization of RRT* paths
│   ├── output_rrt_connect.mp4 # RRT-Connect path visualization
│   ├── output_improved_rrt_connect.mp4 # Improved RRT-Connect path visualization
│── utils/
│   ├── visualization.py    # Helper function to plot and generate videos
│   ├── environment.py      # Code to generate test environments
```

## **Installation & Setup**
### **Step 1: Clone the Repository**
```
git clone https://github.com/<your-github-username>/improved_rrt_connect.git
cd improved_rrt_connect
```

### **Step 2: Install Dependencies**
```
pip install numpy opencv-python matplotlib scipy
```

### **Step 3: Running the Path Planning Algorithms**
Each algorithm can be executed independently:

Run Standard RRT
```
python rrt.py
```
Run RRT*
```
python rrt_star.py
```
Run RRT-Connect
```
python rrt_connect.py
```
Run Improved RRT-Connect
```
python improves_rrt_connect.py
```
## **How the Code Works**
1️⃣ **Map Initialization**
Defines a 600x200 pixel map with obstacles.
Applies clearance and robot radius constraints for safe navigation.
2️⃣ **Path Planning Algorithms**
RRT: Grows a single tree by randomly exploring the space.
RRT*: Refines the paths for optimality by rewiring nodes.
RRT-Connect: Uses bidirectional growth from start and goal.
Improved RRT-Connect: Introduces a midpoint strategy, four-way tree expansion, and goal biasing for faster and more stable pathfinding.
3️⃣ **Visualization**
Uses OpenCV to visualize path generation.
Generates a video output of the final trajectory

## **Experimental Results**
The **Improved RRT-Connect** method significantly **outperforms** other path planning algorithms:

| Start & Goal          | RRT (sec) | RRT* (sec) | RRT-Connect (sec) | Improved RRT-Connect (sec) |
|----------------------|-----------|------------|--------------------|----------------------------|
| (20, 20) → (580, 50) | 0.83      | 1.40       | 0.36               | **0.18**                   |
| (100,100) → (300,50) | 1.08      | 1.35       | 0.10               | **0.03**                   |
| (190,100) → (300,50) | 0.30      | 1.36       | 0.05               | **0.005**                  |
| (100,50) → (500,50)  | 0.52      | 1.51       | 0.34               | **0.07**                   |

🔹 Observations:

- Standard RRT takes longer due to randomized exploration.
- RRT improves path quality* but takes longer due to restructuring.
- RRT-Connect is significantly faster, but sometimes inefficient in tight spaces.
- Improved RRT-Connect achieves the best balance of speed & accuracy.

## **References**
📄 Paper Citation: Hariharasudan Muralidaran, Harshavarthan Varatharajan (2024).
"Path planning for mobile robots with improved RRT-Connect algorithm."

🔗 Related Works:

- LaValle, S. M. (1998). "Rapidly-exploring random trees: A new tool for path planning."
- Karaman, S. & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning."
- Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). "Informed RRT*: Optimal path planning."




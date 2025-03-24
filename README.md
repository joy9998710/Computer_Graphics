# PA1_2024 (Ray Tracing Algorithm)

## 1. Introduction
This project implements a basic **Ray Tracer** that supports:

- Spheres as geometric primitives
- Lambertian (diffuse) and Blinn-Phong (specular) shading models
- Point lights that do not attenuate with distance

The goal is to generate visually realistic images using simple and efficient ray tracing techniques. Although this implementation is not physically accurate, it produces clean and convincing results.

## 2. Environment   
- Language : Python
- Tool : Numpy, PIL

## 3. Workflow   
1. **Scene File Parsing**  
   The program reads a well-formed scene description file. This file defines:
   - Camera parameters
   - A list of spheres with their positions, radii, and material properties
   - Point light sources (with no attenuation)
   - The shading model to be applied (Lambertian or Blinn-Phong)

2. **Ray Generation**  
   Rays are generated from the camera through each pixel in the image plane. Each ray represents the path of light entering the virtual camera.

3. **Ray-Sphere Intersection Testing**  
   For every ray, the program checks for intersections with all spheres in the scene. If an intersection is found, the closest one is selected as the point of interest.

4. **Shading Calculation**  
   Depending on the selected shading model:
   - **Lambertian Shading** computes the diffuse color based on the angle between the surface normal and the light direction.
   - **Blinn-Phong Shading** adds specular highlights by considering the viewer direction and the halfway vector between light and view directions.
   Lighting is calculated using **point lights that do not attenuate with distance**, as specified in the assignment.

5. **Pixel Color Assignment**  
   The final color for each pixel is determined by the shading result and stored in an image buffer.

6. **Image Output**  
   Once all pixels are processed, the image buffer is saved as an output file in `.png` format.

## 4. How to run   
Move into PA1_2024 file
`cd ./CGAssignment/PA1_2024`   

**For Phong Shading**
`python rayTracer.py ./scenes/one-sphere.xml`

**For Lambertian Shading**
`python rayTracer.py ./scenes/four-spheres.xml`

## 5. Results

**Lambertian Shading Example**   
![Lambertian example](./CGAssignment/PA1_2024/scenes/four-spheres.xml.png)

**Phong Shading Example**   
![Phong example](./CGAssignment/PA1_2024/scenes/one-sphere.xml.png)


# PA2_2024 

## 1. Introduction   
This project implements an interactive 3D animation system called the **Cow Roller Coaster**. The objective is to build an intuitive control interface for defining motion trajectories, and animate a 3D cow model using a **Hermite spline curve** for smooth interpolation.

Key features include:
- Picking and positioning a 3D cow model using mouse input
- Vertical and horizontal dragging to place control points
- Interpolation of six user-defined positions using a Hermite spline curve
- Real-time animation of the cow along the path with proper orientation (yaw and pitch)

## 2. Environment   
- Language : Python
- Tools : OpenGL, GLFW, numpy, PIL

## 3. Workflow

1. **Mouse-Based Cow Placement**
   - The user selects a cow model by left-clicking.
   - Six control points are defined via successive clicks.
   - Each click duplicates the cow model at the selected position.

2. **Vertical and Horizontal Dragging**
   - Vertical dragging is performed using ray-plane intersection based on cursor input.
   - Horizontal dragging reuses the fixed dragging plane captured during vertical drag.
   - The plane position is maintained via a global variable (`glob_plane_pos`).

3. **Control Point Recording**
   - On each click, the cow’s position is stored in `glob_cow_pos`.
   - Once all six points are selected, the animation process begins.

4. **Hermite Spline Animation**
   - A Hermite spline curve is constructed using the six control points.
   - The cow moves along the spline based on relative time (via `glfw.get_time()`).
   - Position is calculated using cubic Hermite interpolation.
   - The derivative of the spline is used to determine orientation (direction and pitch).
   - The cow follows the path 3 full cycles and then returns to interactive mode.

5. **Rendering and Display**
   - During the placement phase, static cows are drawn at selected points.
   - During animation, the moving cow is rendered frame-by-frame using the interpolated position and orientation.

## 4. How to run
Move to the PA2_2024 file   
`cd ./CGAssignment/PA2_2024`

Run the SimpleScene.py file   
`python SimpleScene.py`

## 5. Results

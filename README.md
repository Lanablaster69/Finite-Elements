Analyses truss structures in MATLAB via Finite Element Method
This MATLAB script is designed for the analysis of a 2D truss structure. It calculates nodal displacements, reaction forces, and stress in each element of the truss based on user-defined parameters. The script visualizes both the undeformed and deformed shapes of the truss, allowing for a comparative analysis of the structure before and after loading.

## Features

- Input and configuration of nodal coordinates, element connectivity, material properties, boundary conditions, and external forces.
- Calculation of the global stiffness matrix and nodal displacements.
- Calculation of reaction forces at constrained nodes.
- Visualization of undeformed and deformed truss shapes.
- Stress calculation for each element based on deformation.
- Mass efficiency computation of the truss.


Usage
Input Nodal Coordinates: The nodal coordinates must be specified in meters. An example format is:


NodalCoords = [0 0; 1 0; 2 0; 3 0; 2 0.5; 1 1; 0 1.5];
Define Element Connectivity: Specify the pairs of nodes connected by each element:


ConnectingNodes = [1 2; 2 3; 3 4; 4 5; 5 6; 6 7; 1 6; 2 6; 2 5; 3 5];
Material Properties:

Modulus of Elasticity (E):

E = [69e9; 69e9; 69e9; 69e9; 69e9; 210e9; 69e9; 69e9; 69e9; 210e9];
Cross-Sectional Areas (A):

A = [0.0001; 0.00007; 0.00008; 0.00006; 0.0001; 0.000045; 0.000025; 0.00005; 0.00003; 0.00005];
Boundary Conditions: Specify constrained nodes with a vector:


BC = [1; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 1];

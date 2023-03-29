%% Introduction
%{

                 _.--""""''-.
              .-' Created by '.
            .' Spencer         '.
           /  Reznick   .        )
          | 213602792         _  (
          |          .       / \  \
          \         .     .  \_/  |
           \    .--' .  '         /
            \  /  .'____ _       /,
             '/   (\    `)\       |
             ||\__||    |;-.-.-,-,|
             \\___//|   \--'-'-'-'|
              '---' \             |
       .--.          '---------.__)   .-.
      .'   \                         /  '.
     (      '.                    _.'     )
      '---.   '.              _.-'    .--'
           `.   `-._      _.-'   _.-'`
             `-._   '-.,-'   _.-'
                 `-._   `'.-'
               _.-'` `;.   '-._
        .--.-'`  _.-'`  `'-._  `'-.--.
       (       .'            '.       )
        `,  _.'                '._  ,'
          ``                      ``


%}
%{
    Enter coordinates of node in vector form: [0 0;1 0;2 0;3 0;2 0.5;1 1;0 1.5]
Enter the nodes that connect the elements in vector form: [1 2;2 3;3 4;4 5;5 6;6 7;1 6;2 6;2 5;3 5]
Enter the Modulus of Elasticity in vector form for each element in N/m.^2: [210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9]
Enter the cross-sectional area in vector form for each element in m.^2: [0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003]
Enter the length of each element in vector form in m: [1;1;1;1.118;1.118;1.118;1.414;1;1.118;0.5]
Enter the angle of the elements in vector form in degrees: [0;0;0;135;135;135;45;90;45;90]
Enter Boundary condition in vector form with 1 where there is a constraint at the node wrt degrees of freedom: [1;1;0;0;0;0;0;0;0;0;0;0;1;1]
Enter the forces corresponding to the nodes in N: [0;0;0;-3000;0;-3000;0;-3000;0;0;0;0;0;0]
%}
%% Clear and Close
close all
clear all
clc
%% Input Section

%warning=input('Make sure elements are entered in the same order each time you enter a value for elements, ie 1, 2, 3. Press any key to continue');

%Create a matrix that stores the nodal coordinates
NodalCoords =[0 0;1 0;2 0;3 0;2 0.5;1 1;0 1.5];% input("Enter all nodal coordinates (meters) in the format: [x1 y1; x2 y2; ...]");
NumOfNodes = 7;%input("Enter the Total Number of Nodes in the Structure");
ConnectingNodes =[1 2;2 3;3 4;4 5;5 6;6 7;1 6;2 6;2 5;3 5] ;%input("Enter the pairs of nodes connected by elements in the format: [1 2;1 3;2 3;...] where 1, 2, 3 are nodes");

%Create individual vectors for each of the parameters E(e), A(e), L(e), and θ(e)0
E =[210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9;210e9];% input('Enter modulus of elasticity (Pascals) for each element in element order (ie. 1,2,3) in the format: [210e9; 210e9; 69e9; 210e9;...]');
A =[0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003;0.003];% input('Enter the cross-sectional area (m^2) for each element in element order (ie. 1,2,3) in the format: [0.003; 0.15; 0.003;...]');
L =[1;1;1;1.118;1.118;1.118;1.414;1;1.118;0.5];% input('Enter the length (m) of each element in element order (ie. 1,2,3) in the format: [1; 1.25; 0.8;...]');
theta = [0;0;0;135;135;135;45;90;45;90];%input('Enter the angle (degrees) of each element in element order (ie. 1,2,3) in the format: [0; 45; -30;...]');

%Create the Boundary Condition Vector
BC =[1;1;0;0;0;0;0;0;0;0;0;0;1;1];% input("Enter a vector where each node that is constrained is a 1 and each unconstraied node is a 0 in the format: [1;0;0;1;0;1;1;...]"); %must be entered in order

%Create the global force vector
GlobalForce =[0;0;0;-3000;0;-3000;0;-3000;0;0;0;0;0;0];% input("Enter the forces(Newtons) corresponding to each node in the format: [0;0;0;-3000;-3000;0;0;-3000;...] basically[x1;y1;x2;y2...]");
%GlobalForce

%Create the element interconnectivity info matrix 
NumberOfElements = size(ConnectingNodes,1); % # of rows
ElementNumber = [1:NumberOfElements]; % creates an array from 1 to the number of elements
ElementNumberTransp = ElementNumber.'; 
ElementIntInfo = [ElementNumberTransp ConnectingNodes ]; %creates an array were the 1st column is the element number and the next 2 are connecting nodes

% Misc. Variables 
xNodalCoords = NodalCoords(:,1);
yNodalCoords = NodalCoords(:,2);


%% Solution Section
DOF=NumOfNodes*2; %for 2 degrees of freedom per node               MAYBE ISSUE!!

%assembly routine that assembles the global stiffness matrix using the information from the element interconnectivity information matrix
Stiff = Stiff(DOF,xNodalCoords,yNodalCoords,NumberOfElements,ConnectingNodes,E,A,L);

%Reduce the equations                                       CHECK ON THIS
Reduce1 = BC(:,1)==1; %finds 1s in BC (boudary conditions)
Stiff(:,Reduce1)=[]; %row and column deletion
Stiff(Reduce1,:)=[] %row and column deletion
GlobalForce(Reduce1,:)=[] %row deletion for global force vector           

%Solve for the nodal displacements
Displacements = Stiff\GlobalForce;%inv(Stiff)*GlobalForce;
fprintf('\nDisplacement vector is:\n');
disp(Displacements);
%% Post-Processing Section   








%{
%Solve for the reaction forces
%disp("trial")
%Stiff
%Stiff*Displacements
%Reaction = BC(:,1)==1;
REDUCE2 = BC(:,1)==0; %Finds where there are no constraints to delete later
%Stiff(:,REDUCE2) =[] %
Stiff(REDUCE2,:) =[] %makes the stiffness matrix the right size for the react. Force matrix
ReactionForces = Stiff*Displacements;
fprintf('\Reaction Force vector is:\n');
disp(ReactionForces);
%}



%Calculate the stress in each element

%Plot the undeformed and deformed (with a scale factor) shapes of the structure

ScaleFactor = input(['Enter the desired scale factor as a single number: \n'])

%unDeformed
for n = 1:NumberOfElements
   xUndeformed = [NodalCoords(ConnectingNodes(n,1),1) NodalCoords(ConnectingNodes(n,2),1)];
   yUndeformed = [NodalCoords(ConnectingNodes(n,1),2) NodalCoords(ConnectingNodes(n,2),2)];
   plot(xUndeformed,yUndeformed,'c')
   hold on
end


% Inserts a row of zero to Displacements if BC=1
for i = 1:length(BC)
    if BC(i) == 1
        Displacements = [Displacements(1:i-1,:); zeros(1,size(Displacements,2)); Displacements(i:end,:)];
    end
end

%Displacements

DispRes = reshape(Displacements,2,[]); %resapes Displacements to nx2 
DispRes = DispRes.'
DispRes = DispRes*ScaleFactor;% Multiplies scalefactor by displacements to male them more pronounced

DeformedCoords = DispRes + NodalCoords; %add the deformation to the original coordinates

%Deformed
for n = 1:NumberOfElements
    xDeformed = [DeformedCoords(ConnectingNodes(n,1),1) DeformedCoords(ConnectingNodes(n,2),1)];
    yDeformed = [DeformedCoords(ConnectingNodes(n,1),2) DeformedCoords(ConnectingNodes(n,2),2)];
    plot(xDeformed,yDeformed,'k')
    hold on
end
title('Deformed vs unDeformed Truss Structure')
%legend(unDeformed,Deformed)


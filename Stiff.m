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

% Function that generates the element stiffness matrix in global coordinates given the element paramete
function [Stiff] = ElementStiffness(DOF,xNodalCoords,yNodalCoords,NumberOfElements,ConnectingNodes,E,A,L) 
Stiff = zeros(DOF); %initializes a zero matrix

for i = 1:NumberOfElements
    CN = ConnectingNodes(i,:);
    elementDOF = [CN(1)*2-1 CN(1)*2 CN(2)*2-1 CN(2)*2];
    l=xNodalCoords(CN(2))-xNodalCoords(CN(1)); %dist btw x coords
    h=yNodalCoords(CN(2))-yNodalCoords(CN(1)); %dist btw y coords
    L =  sqrt(l.^2+h.^2); %length of element IDK why I can't just call this "L" but this works
    e= E(i,:);
    a= A(i,:);
    k = ((e*a)/L)*[l^2 l*h -(l^2) -l*h; l*h h^2 -l*h -(h^2); -(l^2) -l*h l^2 l*h; -l*h -(h^2) l*h h^2];
    Stiff(elementDOF,elementDOF) = Stiff(elementDOF,elementDOF)+k;
    Stiff
end

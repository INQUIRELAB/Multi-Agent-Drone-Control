
% Define parameters of the arc.
xCenter = 2;
yCenter = 1; 
radius = 5;
% Define the angle theta as going from 30 to 150 degrees in 100 steps.
theta = linspace(30, 150, 100);
% Define x and y using "Degrees" version of sin and cos.
x = radius * cosd(theta) + xCenter; 
y = radius * sind(theta) + yCenter; 
% Now plot the points.
plot(x, y, 'b-', 'LineWidth', 2); 
axis equal; 
grid on;
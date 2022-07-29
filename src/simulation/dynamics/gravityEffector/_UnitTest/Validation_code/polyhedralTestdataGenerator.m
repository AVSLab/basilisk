% Define Eros parameters
G = 6.67*1e-11;
mu = 4.46275472004 * 1e5;
rotPeriod = 5.27*3600;
omega = [0,0,2*pi/rotPeriod]';
filenamePoly = 'EROS856Vert1708Fac.txt';
[vert, facets, xyz, order, vol_ast] = read_shape(filenamePoly);
rho = mu/(G*vol_ast*1e9);
xyz_poly = xyz*1e3;
order_poly = order;

% Define spacecraft orbit
pos = [-10559,22699,23001]';
vel = [4.6504,1.4116,0.74723]';
x0 = [pos;
    vel];

% Define simulation time
t0 = 0;
tf = 24*3600;
dt = 300;
tspan = linspace(t0,tf,tf/dt+1);

% Define ode simulation options
ode_options = odeset('RelTol',1E-12, 'AbsTol',1E-12);
    
% LAUNCH RUNGE-KUTTA ALGORITHM ODE45
[t, x] = ode45(@(t,x) dynamics(t, x, omega, rho, G,  xyz_poly,...
    order_poly), tspan, x0, ode_options);
plot3(x(:,1)/1e3,x(:,2)/1e3,x(:,3)/1e3)
writematrix([t,x],'polyTestData.csv','Delimiter',',')

function f = dynamics(t, x, omega, rho, G, xyz_poly, order_poly)

% Preallocate time derivatives vector
f = zeros(6,1);

% Obtain position derivatives
f(1:3,1) = x(4:6,1);

acc = computePolyAcc(x(1:3,1), xyz_poly, order_poly, rho, G);
    
% Obtain velocities time derivatives
f(4:6) = acc - 2*cross(omega, x(4:6,1)) - cross(omega,cross(omega,x(1:3,1)));
end

function [vert, facets, xyz, order, vol_ast] = read_shape(filename)
% Open shape file and display screen info
fprintf('---- Reading Shape File... ----\n')
fid = fopen(filename);

% Extract parts of filename
[pathstr, name, ext] = fileparts(filename);

% Get first line, convert to vector format
line1 = str2num(fgetl(fid));

% Obtain number of polyhedra vertices and facets
vert = line1(1);
facets = line1(2);

% Check if it is a a .txt file or a .tab file
if strcmp(ext, '.txt')
    % Preallocate matrices and vector corresponding to vertex positions,
    % facets topology and facet polyhedra volume
    xyz = zeros(vert, 3);
    order = zeros(facets, 3);
    vol_facet = zeros(facets, 1);
    
    % Loop through all vertices to fill vertex positions matrix reading
    % filename lines
    for i = 1:vert;
        xyz(i,:) = str2num(fgetl(fid));
    end
    
    % Loop through all facets to fill facets topology matrix reading
    % filename lines. Also, compute single facet volume and store it.
    for i = 1:facets;
        order(i,:) = str2num(fgetl(fid));
        vol_facet(i,1) = abs(dot(cross(xyz(order(i,1),:), xyz(order(i,2),:)),...
            xyz(order(i,3),:)))/6;
    end
    
elseif strcmp(ext, '.tab')
    % Preallocate matrices and vector corresponding to vertex positions,
    % facets topology and facet polyhedra volume
    xyz = zeros(vert, 4);
    order = zeros(facets, 4);
    vol_facet = zeros(facets, 1);
    
    % Loop through all vertices to fill vertex positions matrix reading
    % filename lines
    for i = 1:vert;
        xyz(i,:) = str2num(fgetl(fid));
    end
    
    % Extract only vertex positions, not vertex index numeration
    xyz = xyz(:, 2:4);
    
    % Loop through all facets to fill facets topology matrix reading
    % filename lines. Also, compute single facet volume and store it.
    for i = 1:facets;
        order(i,:) = str2num(fgetl(fid));
        vol_facet(i,1) = abs(dot(cross(xyz(order(i,2),:), xyz(order(i,3),:)),...
            xyz(order(i,4),:)))/6;
    end
    
    % Extract only facets vertex index, not facet index numeration
    order = order(:, 2:4);
    
else
    % Display information on screen if filename format is not adequate
    fprintf('Error, entered asteroid shape filename format is not .txt or .tab\n')
end

% Compute total asteroid volume
vol_ast = sum(vol_facet);

% Close filename
fclose(fid);
end



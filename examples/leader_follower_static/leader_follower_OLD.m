%% Leader-follower with static topology
% Paul Glotfelter
% 3/24/2016

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 700;

%% Set up the Robotarium object

%Get Robotarium object and set the save parameters
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 2;

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();

%% Create the desired Laplacian

%Graph laplacian
followers = -completeGL(N-1);
L = zeros(N, N);
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1;
L(2, 1) = -1;
L(1, 1) = -1;

%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 0;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.0;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 1, 'AngularVelocityLimit', 2);
% Single-integrator barrier certificates
%si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.00);
% Single-integrator position controller
si_pos_controller = create_si_position_controller();
%distance = zeros(iterations, 2);


vel = [0,0,0];
vel(1,4) = -10;
vel(1,5) = -10;


for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    %distance(t, 2) = sqrt((x(1,1)-x(1,2))^2+(x(2,1)-x(2,2))^2);
    %distance(t, 1) = t;
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        %Original version
%         neighbors = topological_neighbors(L, i);
%         for j = neighbors
%             dxi(:, i) = dxi(:, i) + ...
%                 formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
%         end
        

        % Version 2 dxi(:, i) = formation_control_gain*(norm(x(1:2, 1) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, 1) - x(1:2, i));
        
        %TEST Version
%         add = zeros(2,1);
%         add(1,1) = add(1,1) + 0.05;
%         add(2,1) = add(2,1) + 0.05;
%         x(1:2, 1) =  x(1:2, 1)+ add(1:2,1);
        dxi(:, i) = formation_control_gain*(norm(x(1:2, 1) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, 1) - x(1:2, i));
        distance = norm(x(1:2, 1) - x(1:2, 2));
    end
    
    %% Make the leader travel between waypoints
%     
    switch state
        case 0
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [7 ; 7]);
            if(norm(x(1:2, 1) - [7 ; 7]) < 0.05)
                state = 1;
            end
        case 1
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [-7 ; 7]);
            if(norm(x(1:2, 1) - [-7 ; 7]) < 0.05)
                state = 2;
            end
        case 2
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [-7 ; -7]);
            if(norm(x(1:2, 1) - [-7 ; -7]) < 0.05)
                state = 3;
            end
        case 3
            dxi(:, 1) = si_pos_controller(x(1:2, 1), [7 ; -7]);
            if(norm(x(1:2, 1) - [7 ; -7]) < 0.05)
                state = 0;
            end
    end
    
%     switch state
%         
%         case 0
%             dxi(:, 1) = si_pos_controller(x(1:2, 1), [1.5 ; 1.5]);
%             if(norm(x(1:2, 1) - [1.5 ; 1.5]) < 0.05)
%                 disp('Destanation reached');
%                 disp(t);
%                 disp('State 1');
%                 
%                 if t == 2000
%                     state = 1;
%                 end
%                 
%             end
%             
%         case 1
%             dxi(:, 1) = si_pos_controller(x(1:2, 1), [-1.5 ; 1.5]);
%             disp('State 2');
%             if(norm(x(1:2, 1) - [-1.5 ; 1.5]) < 0.05)
%                 disp('State 2 reached');
%             end
%     end
                
    vel(t,1) = t;
    vel(t,2) = x(1,2); %x
    vel(t,3) = x(2,2); %y
    %disp(vel(t,2:3));
    disp(t);
    vel(t+1,4) = x(1,2); %prev x
    vel(t+1,5) = x(2,2); %prev y
    vel(t,6) = (abs(vel(t,2)) - abs(vel(t,4)))^2 + (abs(vel(t,3)) - abs(vel(t,5)))^2; 
    vel(t,7) = distance;
    disp(vel(t,6));
    
    %% Use barrier certificate and convert to unicycle dynamics
    %dxi = si_barrier_cert(dxi, x);
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %Iterate experiment
    r.step();
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
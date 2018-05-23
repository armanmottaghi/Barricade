%% Leader-follower with static topology
%LAP 2

%% Experiment Constants\
  


start_clock =clock;
fprintf('Start Time: %.2f hours, %.2f minutes and %.2f seconds.\n', start_clock(4), start_clock(5),start_clock(6) );
%Run the simulation for a specific number of iterations
iterations = 20000;

%% Set up the Robotarium object

%Get Robotarium object and set the save parameters
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = 15;

NOD = 5;
NOE = N-NOD;
% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
r = rb.set_number_of_agents(N);
r = rb.set_save_data(false);
CapturedEnemy = [];
r = rb.set_CapturedEnemies(CapturedEnemy);
r = rb.set_number_of_enemies(NOE);
r = rb.set_number_of_defenders(NOD);
r = rb.build();
%% Create the desired Laplacian

%Graph laplacian
L_Enemies = DiagLapMatrix(N-1, -1);
L = zeros(N, N);
L(2:N, 2:N) = L_Enemies;
L(1, 1) = 1;

%Initialize DefenderInfoocity vector
dxi = zeros(2, N);

%State for leader
state = 0;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.0;
CompoundBuildingRadius = 0.7;
%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 1, 'AngularVelocityLimit', 2);
% Single-integrator barrier certificates
%si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.005);
% Single-integrator position controller
si_pos_controller = create_si_position_controller();
%distance = zeros(iterations, 2);


DefenderInfo = [0,0,0];
EnemyInfo = [0,0,0];

x = r.get_poses();
target_condition = zeros(3,N);

% for i = 2:N
%     target_condition(1:2,i) = (-1 * x(1:2,i));
%     %target_condition(1:2,i) =[0;0];
%     target_condition(3,i) = x(3,i);
% end

inc = (iterations/60);
start = 1;
timer= 40;

AIA = zeros (N,16);
Captured = 0;

%figure2 = figure;
%figure2handle = gca;
CaptureArray = zeros(1,10);
for t = 1:iterations
    
    %% Generate Random X,Y destination coordinates after every 'inc' intervals
    if t == start
        for i = (NOD+1):N
            ap=2*pi*rand;
            rp=sqrt(rand);
            xp=(10*rp)*cos(ap);
            yp=(10*rp)*sin(ap);
            target_condition(1:2,i) = [xp;yp];
        end
        
        start = start + 300;
    end
    
    %% Get Poses
  
    x = r.get_poses();
    CapturedEnemy = r.get_captured_enemy();
    
    %% New Agent Array Info
    
    Violation = 0;
    for i = 1:N
        
        CapturedEnemy_Size = size(CapturedEnemy);
        NumberOfCapturedEnemies = CapturedEnemy_Size(1);
        AlreadyCaptured = 0;
        
        for k = 1:NumberOfCapturedEnemies
            if i == CapturedEnemy(k)
                AlreadyCaptured = 1;
            end
        end
        
        if AlreadyCaptured ~= 1
            
            AIA(i,1) = i;

            if i <= NOD
                AIA(i,2) = 0;
            else
                AIA(i,2) = 1;
            end

            AIA(i,3) = x(1,i); % x element of position vector of agent i
            AIA(i,4) = x(2,i); % y element of position vector of agent i

            AIA(i,7) = (0 - AIA(i,5)); % distance vector of enemy i to (0,0) {{ x element }} 
            AIA(i,8) = (0 - AIA(i,6)); % distance vector of enemy i to (0,0) {{ y element }} 
            
            AIA(i,9) = (x(1,i) - AIA(i,5)) ; % relative velocity vector of enemy i {{ x element }} 
            AIA(i,10) = (x(2,i) - AIA(i,6)); % relative velocity vector of enemy i {{ y element }} 
            
            %ab2v([0,0],[AIA(i,9), AIA(i,10)],[AIA(i,7), AIA(i,8)], figure2handle);  
            
            
            % Column 11: Compute angle alpha: angle between the distance vector
            % and rel velocity vector
            if ((norm([AIA(i,7),AIA(i,8)]))*(norm([AIA(i,9),AIA(i,10)]))) ~= 0
                AIA(i,11) = rad2deg(acos(dot([AIA(i,7),AIA(i,8)],[AIA(i,9),AIA(i,10)])/((norm([AIA(i,7),AIA(i,8)]))*(norm([AIA(i,9),AIA(i,10)]))))); % Amgle between distance to origin vector and rel velocity vector
            else
                AIA(i,11) = 0;
            end

            % Column 12: compute critical angle of agennt i - it's the angle
            % that agent i's alpha angle must be within in order for agent i to
            % pass the target compound

            if (norm([AIA(i,7),AIA(i,8)])) ~= 0
                AIA(i,12) = rad2deg(atan(CompoundBuildingRadius/(norm([AIA(i,7),AIA(i,8)])))); % Critial angle
            else
                AIA(i,12) = 0;
            end

            %Column 13: Determining whether enemy i has violated the safe space or not
            if (norm(x(1:2,i)) < 4) && (AIA(i,2) == 1)
                Violation = Violation + 1;
                AIA(i,13) = 1;
            else
                AIA(i,13) = 0;
            end

           %Finding the true angle: it could be either theta or 360-theta...
            if abs(AIA(i,11)) > 180
                Alpha = 360 - abs(AIA(i,11));
            else
                Alpha = abs(AIA(i,11));
            end
            
            alpha_critical = AIA(i,12);
            alpha_w = (alpha_critical - Alpha)/(alpha_critical);
            
            %Column 14: Urgent or Not Urgent 
            if AIA(i,13) == 1 && (AIA(i,2) == 1)
                
                if (alpha_w <= 1) && (alpha_w > 0)
                elseif (alpha_w > 1)
                elseif (alpha_w == 0)
                    alpha_w = 0.00001;
                elseif (alpha_w < 0) && (alpha_w > -1)
                    alpha_w = alpha_w + 2;
                elseif (alpha_w <= -1)
                    alpha_w = -1 * alpha_w;
                end
                  
                EnemyDistance = norm([AIA(i,3), AIA(i,4)]);
                urgency = 1/((EnemyDistance/10)^(1.5)*(alpha_w));
                
                AIA(i,14) = EnemyDistance;
                AIA(i,15) = urgency;
            else
                AIA(i,14) = 0;
                AIA(i,15) = 0;
            end

            % Column 15 is the status of agent (defender: chasing or default...
            % enemy: being chased or free)
            
        else
            AIA(i,1) = i;
            AIA(i,2) = 2;
            AIA(i,3) = 0;
            AIA(i,4) = 0;
            AIA(i,5) = 0;
            AIA(i,6) = 0;
            AIA(i,7) = 0;
            AIA(i,8) = 0;
            AIA(i,9) = 0;
            AIA(i,10) = 0;
            AIA(i,11) = 0;
            AIA(i,12) = 0;
            AIA(i,13) = 0;
            AIA(i,14) = 0;
            AIA(i,15) = 0;
        end
    end 


    %% Controller for enemy agents
    for i = (NOD+1):N
        dxi(1:2, i) = si_pos_controller(x(1:2, i), target_condition(1:2, i));
    end
  
    
    %% New Controller for Defender Agents
    ViolationArray = [];
    EnemyIndex = NOD+1;
    q = 1;
    for i = EnemyIndex:N
        if AIA(i,2) ~= 2 && AIA(i,13)~=0 && AIA(i,16) == 0
            ViolationArray(q,1) = i; %index
            ViolationArray(q,2) = AIA(i,3); %x
            ViolationArray(q,3) = AIA(i,4); %y
            ViolationArray(q,4:6) = AIA(i, 14:16);
            q = q + 1;
        end
    end
    
    sizevar = size(ViolationArray);
    ViolationArraySize = sizevar(1); 
    
    if ViolationArraySize ~= 0
        
        if ViolationArraySize == 1
            SortedViolationArray = sortrows (ViolationArray, 3);
        else
            SortedViolationArray = sortrows (ViolationArray, 3);
            SortedViolationArray = flipud(SortedViolationArray);
        end

        q = 1;
        for i = 1:NOD
            if AIA(i,16) == 0
                AvailableDefendersArray(q, 1) =  i;
                AvailableDefendersArray(q,2) = AIA(i,3); %defender x 
                AvailableDefendersArray(q,3) = AIA(i,4); %defender y
                q = q + 1;
            end
        end
    
        sizevar = size(AvailableDefendersArray);
        AvailableDefenders = sizevar(1);
        ADN = AvailableDefenders;
        sizevar = size(SortedViolationArray);
        ViolatorsNumber = sizevar(1);
    
        if AvailableDefenders ~= 0 
            for i = 1:ViolatorsNumber
                ViolatorX = SortedViolationArray(i,2);
                ViolatorY = SortedViolationArray(i,3);    

                if AvailableDefenders ~= 0
                    for j = 1:AvailableDefenders
                        DefenderX = AvailableDefendersArray(j,2);
                        DefenderY = AvailableDefendersArray(j,3);
                        DefenderIndex = AvailableDefendersArray(j, 1);

                        RelativeDistance = norm([(ViolatorX - DefenderX),(ViolatorY - DefenderY)]);
                        DistanceRankingArray(j,1) = DefenderIndex;
                        DistanceRankingArray(j,2) = RelativeDistance;
                    end

                    DistanceRankingArray = sortrows(DistanceRankingArray, 2);
                    OptimalDefenderIndex = DistanceRankingArray(1,1);
                    SortedViolationArray(i,6) = OptimalDefenderIndex;

                    for k = 1:AvailableDefenders
                        if AvailableDefendersArray(k, 1) == OptimalDefenderIndex
                            thisOne = k;
                        end
                    end
                    
                    AvailableDefendersArray(thisOne,:) = [];
                    
                    sizevar = size(AvailableDefendersArray);
                    AvailableDefenders = sizevar(1); 

                else
                    i = ViolatorsNumber;
                end
            end
            
           

            sizevar = size(SortedViolationArray);
            ViolatorsNumber = sizevar(1);

            %AssignmentArray = zeros(1:ADN, 1:2);
            if ViolatorsNumber == 1
                AssignmentArray(1, 1) = SortedViolationArray(1, 1);
                AssignmentArray(1, 2) = SortedViolationArray(1, 6);
            elseif ViolatorsNumber > 1
                AssignmentArray(1:ViolatorsNumber, 1) = SortedViolationArray(1:ViolatorsNumber, 1);
                AssignmentArray(1:ViolatorsNumber, 2) = SortedViolationArray(1:ViolatorsNumber, 6);
            end

            sizevar = size(AssignmentArray);
            AssignmentArraySize = sizevar(1);

            for j = 1:AssignmentArraySize
                En = AssignmentArray(j,1);
                Df = AssignmentArray(j,2);
                if Df ~= 0
                    
                    AIA(En,16) = Df;
                    AIA(Df,16) = En;
                end
            end      
                  
        end
        
    end
        %% Assignment Section
        

        
    for k = 1:NOD
        if AIA(k,16) == 0
            dxi(1:2, k) = formation_control_gain*(norm([0;0] - x(1:2, k))^2)*([0;0] - x(1:2, k));
        else
            Target_Enemy = AIA(k,16);
            dxi(1:2, k) = formation_control_gain*(norm(x(1:2, Target_Enemy) - x(1:2, k))^2 -  desired_distance^2)*(x(1:2, Target_Enemy) - x(1:2, k));
        end
    end
    
%  
    %% Capturing Procedure
   
    row = 1;
    for l = 1:NOD
        if AIA(l,16) ~= 0
            CaptureArray(row, 1) = l;
            CaptureArray(row, 2) = AIA(l,16);
            EnemyInd = AIA(l,16);
            DefenderPos = [AIA(l,3),AIA(l,4)];
            EnemyPos = [AIA(EnemyInd,3),AIA(EnemyInd,4)];
            Distance = norm([(DefenderPos(1) - EnemyPos(1)),(DefenderPos(2) - EnemyPos(2))]);
            CaptureArray(row, 3) = Distance;
            
            if CaptureArray(row, 4) ~= 0
            else
                CaptureArray(row, 4) = 10;
            end
            row = row + 1;
        end
    end
    
    if (exist('CaptureArray') == 1)
        sizevar = size(CaptureArray);
        CaptureArraySize = sizevar(1); 
        
        if CaptureArraySize > 0 && CaptureArray(1,1) ~= 0
             
            for y = 1:CaptureArraySize
                if CaptureArray(y, 3) < 0.21 && (CaptureArray(y, 4) > 1)
                    CaptureArray(y, 4) = CaptureArray(y, 4) - 1;
                    fprintf('Defender %.0f is Chasing Enemy: %.0f at Timer: %.0f when the Distance is %.5f ! \n', CaptureArray(y, 1), CaptureArray(y, 2),CaptureArray(y, 4),  CaptureArray(y, 3));
                elseif CaptureArray(y, 3) < 0.21 && (CaptureArray(y, 4) == 1)
                       
                CapturedEnemy_Size = size(CapturedEnemy);
                TotalCapturedEnemy = CapturedEnemy_Size(1);
                fprintf('Enemy %.0f is captured! Total camptured enemy agents: %.0f! Distance to enemy is %.5f! \n',CaptureArray(y, 2), TotalCapturedEnemy, CaptureArray(y, 3));
                r.set_CapturedEnemies_New(CaptureArray(y, 2));
                AIA((CaptureArray(y, 2)), :) = [];
                AIA((CaptureArray(y, 1)), 16) = 0;
                Captured =+ 1;
                
                end
            end      
            
            
        end
    end

    
    CapturedEnemy = r.get_captured_enemy();
    
    CapturedEnemy_Size = size(CapturedEnemy);
    TotalCapturedEnemy = CapturedEnemy_Size(1);
%     
%     if (TotalCapturedEnemy) == (N-NOD)
%         end_clock = clock;
%         minutes = end_clock(5) - start_clock(5);
%         sec = end_clock(6) - start_clock(6);
%         fprintf('All the enemies are captured!!\n');
%         fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
%         fprintf('Duration: %.0f iterations! \n', t);
%         r.call_at_scripts_end();
%         break;
%     end
    
  
    
    for i = (NOD+1):N
        CapturedEnemy_Size = size(CapturedEnemy);
        NumberOfCapturedEnemies = CapturedEnemy_Size(1);
        AlreadyCaptured = 0;

        if NumberOfCapturedEnemies ~= 0
            for k = 1:NumberOfCapturedEnemies
                if i == CapturedEnemy(k)
                    AlreadyCaptured = 1;
                end
            end

            if AlreadyCaptured ~= 1
                AIA(i,5) = x(1,i); % previous x element of position vector of agent i
                AIA(i,6) = x(2,i); % previous y element of position vector of agent i
            else
                AIA(i,5) = 0;
                AIA(i,6) = 0;
            end
        end   
    end   
    
    for i = 1:NOD

        AIA(i,5) = x(1,i); % previous x element of position vector of agent i
        AIA(i,6) = x(2,i); % previous y element of position vector of agent i
  
    end
    
    
    
    %% Use barrier certificate and convert to unicycle dynamics
    %dxi = si_barrier_cert(dxi, x);
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send DefenderInfoocities to agents
    
    %Set DefenderInfoocities
    r.set_velocities(1:N, dxu);
    r.step();
    %figure1handle = gca;
    %fprintf('Figure 1 handle is %0.2f !', figure1handle);
end

end_clock = clock;
minutes = end_clock(5) - start_clock(5);
sec = end_clock(6) - start_clock(6);

fprintf('%.0f enemies are captured!!\n',size(CapturedEnemy));
fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
fprintf('Duration: %.0f iterations! \n', t);

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
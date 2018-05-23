function Return = Simulation(DefenderController,EnemyController,EnvironmentalParameters)

display('Initializing MATLAB simulator')

paths = {'utilities/transformations', ...
    'utilities/barrier_certificates', 'utilities/misc', ...
    'utilities/graph', 'utilities/controllers', 'patch_generation'}; 

path_local_sim_init = pwd;
warned = 0;

for i = 1:length(paths) 
   addpath(strcat(path_local_sim_init, '/', paths{i})); 
   if(exist(paths{i}) ~= 7) 
      warning('The path %s was not correctly added.  Make sure that you are in the directory of the simulator!', paths{i}); 
      warned = 1;
   end
end

if(warned == 0) 
   addpath(pwd);
   display('MATLAB simulator initialized successfully!')
end



start_clock = clock;
%fprintf('Start Time: %.2f hours, %.2f minutes and %.2f seconds.\n', start_clock(4), start_clock(5),start_clock(6) );
gamma = DefenderController.gamma;
lambda = DefenderController.lambda;
DefTimer = DefenderController.DefTimer;

iterations = EnvironmentalParameters.iterations;
NOD = EnvironmentalParameters.NOD;
NOE = EnvironmentalParameters.NOE;
boundary_points = EnvironmentalParameters.boundary_points;
boundaries = EnvironmentalParameters.boundaries;
WatchZoneRadius = EnvironmentalParameters.WatchZoneRadius;
CompoundRadius = EnvironmentalParameters.CompoundRadius;
WaitIterationTime = EnvironmentalParameters.WaitIterationTime;
RotationMode = zeros(2, NOD);
CircularRotationInc = 360/NOD;


if DefenderController.type == 1 && DefenderController.DefenderControlRate == 0
    PatrolRadius = DefenderController.Def_radius_init ;

    for n = 1:NOD
        tt = (n-1)*CircularRotationInc;
        xx = PatrolRadius.*cos(deg2rad(tt));
        yy = PatrolRadius.*sin(deg2rad(tt));
        RotationMode(1:2,n) = [xx;yy];
    end
    
    DefenderController.InitialPose = RotationMode;
else
    RotationMode(1:2,NOD) = [0;0];
    DefenderController.InitialPose = RotationMode;
end      


%% Set up the Robotarium object
rb = RobotariumBuilder();
N = NOD + NOE;
VelocityAgentArray = zeros(iterations,N);

% Set the number of agents and whether we would like to save data.  Then,
%% build the Robotarium simulator object!
ra = rb.set_boundary(boundaries);
ra = rb.set_boundary_points(boundary_points);
ra = rb.set_number_of_agents(N);
ra = rb.set_number_of_enemies(NOE);
ra = rb.set_number_of_defenders(NOD);
ra = rb.set_save_data(false);
CapturedEnemy = [];
ra = rb.set_CapturedEnemies(CapturedEnemy);
ra = rb.set_EnvironmentalParameters(EnvironmentalParameters);
ra = rb.set_DefenderController(DefenderController);
ra = rb.set_EnemyController(EnemyController);
ra = rb.build();

%%Initialize DefenderInfoocity vector
dxi = zeros(2, N);
DefenderVelocityLimit = DefenderController.DefenderVelocityLimit;
EnemyVelocityLimit = EnemyController.EnemyVelocityLimit;
%% Grab tools we need to convert from single-integrator to unicycle dynamics
% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2();

%% Single-integrator position controller
EnemyControllerFun = create_si_position_controller('VelocityMagnitudeLimit', EnemyVelocityLimit);
DefenderControllerFun = create_si_position_controller('XVelocityGain', DefenderController.XVelocityGain, 'YVelocityGain', DefenderController.YVelocityGain,'VelocityMagnitudeLimit', DefenderVelocityLimit);

%% Get Poses
x = ra.get_poses();


%% Enemy Controller Mode

target_condition = zeros(3,N);

if EnemyController.type == 2
    %Cross movement
    for rq = (NOD+1):N
        target_condition(1:2,rq) = (-1 * x(1:2,rq));
        target_condition(3,rq) = x(3,rq);
    end
    %Specific location movement
elseif EnemyController.type == 3
    dirVec = [EnemyController(1,2), EnemyController(1,3)];
    for rq = (NOD+1):N
        dirVec = dirVec';
        target_condition(1:2,rq) = dirVec;
        target_condition(3,rq) = x(3,rq);
    end
    

end


%WatchZoneRadius = 4;

start2= 1;
start = 1;

AIA = zeros (N,21);

CaptureArray = zeros(1,4);

t = 1;
iterations = 0;

while t == 1
    
    
    iterations = iterations + 1;
    disp(iterations);
    %% Generate Random X,Y destination coordinates after every 'inc' intervals
    
    
    if EnemyController.type == 1
        RandomMoveRate = EnemyController.EnemyControlRate;

        if t == start
            
            for u = (NOD+1):N
                ap=2*pi*rand;
                rp=sqrt(rand);
                xp=(10*rp)*cos(ap);
                yp=(10*rp)*sin(ap);
                target_condition(1:2,u) = [xp;yp];
            end

            start = start + RandomMoveRate;
            
        end
        
    end
    
    
    %% Get Poses
    x = ra.get_poses();
    CapturedEnemy = ra.get_captured_enemy();

    %% New Agent Array Info
    
    Violation = 0;
    for r = 1:N
        
        CapturedEnemy_Size = size(CapturedEnemy);
        NumberOfCapturedEnemies = CapturedEnemy_Size(1);
        AlreadyCaptured = 0;
        
        for k = 1:NumberOfCapturedEnemies
            if r == CapturedEnemy(k)
                AlreadyCaptured = 1;
            end
        end
        
        if AlreadyCaptured ~= 1
            
            AIA(r,1) = r;

            if r <= NOD
                AIA(r,2) = 0;
            else
                AIA(r,2) = 1;
            end

            AIA(r,3) = x(1,r); % x element of position vector of agent r
            AIA(r,4) = x(2,r); % y element of position vector of agent r
            
     
            AIA(r,7) = (0 - AIA(r,5)); % distance vector of enemy r to (0,0) {{ x element }} 
            AIA(r,8) = (0 - AIA(r,6)); % distance vector of enemy r to (0,0) {{ y element }} 
            
            AIA(r,9) = (x(1,r) - AIA(r,5)) ; % relative velocity vector of enemy r {{ x element }} 
            AIA(r,10) = (x(2,r) - AIA(r,6)); % relative velocity vector of enemy r {{ y element }} 
            
            AIA(r,19) = norm(AIA(r,9), AIA(r,10));
            %ab2v([0,0],[AIA(r,9), AIA(r,10)],[AIA(r,7), AIA(r,8)], figure2handle);  
            
            
            % Column 11: Compute angle alpha: angle between the distance vector
            % and rel velocity vector
            if ((norm([AIA(r,7),AIA(r,8)]))*(norm([AIA(r,9),AIA(r,10)]))) ~= 0
                AIA(r,11) = rad2deg(acos(dot([AIA(r,7),AIA(r,8)],[AIA(r,9),AIA(r,10)])/((norm([AIA(r,7),AIA(r,8)]))*(norm([AIA(r,9),AIA(r,10)]))))); % Amgle between distance to origin vector and rel velocity vector
            else
                AIA(r,11) = 0;
            end

            % Column 12: compute critical angle of agennt r - it's the angle
            % that agent r's alpha angle must be within in order for agent r to
            % pass the target compound

            if (norm([AIA(r,7),AIA(r,8)])) ~= 0
                AIA(r,12) = rad2deg(atan(CompoundRadius/(norm([AIA(r,7),AIA(r,8)])))); % Critial angle
            else
                AIA(r,12) = 0;
            end

            %Column 13: Determining whether enemy r has violated the safe space or not
            if (norm(x(1:2,r)) < WatchZoneRadius) && (AIA(r,2) == 1)
                Violation = Violation + 1;
                AIA(r,13) = 1;
            else
                AIA(r,13) = 0;
            end

           %Finding the true angle: it could be either theta or 360-theta...
            if abs(AIA(r,11)) > 180
                Alpha = 360 - abs(AIA(r,11));
            else
                Alpha = abs(AIA(r,11));
            end
            
            alpha_critical = AIA(r,12);
            alpha_w = (alpha_critical - Alpha)/(alpha_critical);
     
            %Column 14: Urgent or Not Urgent 
            if AIA(r,13) == 1 && (AIA(r,2) == 1)
                
                if (alpha_w <= 1) && (alpha_w > 0)
                elseif (alpha_w > 1)
                elseif (alpha_w == 0)
                    alpha_w = 0.00001;
                elseif (alpha_w < 0) && (alpha_w > -1)
                    alpha_w = alpha_w + 2;
                elseif (alpha_w <= -1)
                    alpha_w = -1 * alpha_w;
                end
                  
                EnemyDistance = norm([AIA(r,3), AIA(r,4)]);

                urgency = 1/((EnemyDistance/10)^(1.5)*(alpha_w));
                
                AIA(r,14) = EnemyDistance;
                AIA(r,15) = urgency;
            else
                AIA(r,14) = 0;
                AIA(r,15) = 0;
            end

            % Column 16 is the status of agent (defender: chasing or default...
            % enemy: being chased or free)
            
            if r <= NOD
                if AIA(r,16) ~= 0
                    Enem_ID = AIA(r,16);
                    if AIA(Enem_ID,13) == 0
                        AIA(r,16) = 0;
                        AIA(Enem_ID,16) = 0;
                    end
                end
            end
            
        else
            
            AIA(r,1) = r;
            AIA(r,2) = 2;
            AIA(r,3) = -1;
            AIA(r,4) = -1;
            AIA(r,5) = -1;
            AIA(r,6) = -1;
            AIA(r,7) = -1;
            AIA(r,8) = -1;
            AIA(r,9) = -1;
            AIA(r,10) = -1;
            AIA(r,11) = -1;
            AIA(r,12) = -1;
            AIA(r,13) = -1;
            AIA(r,14) = -1;
            AIA(r,15) = -1;
            AIA(r,16) = -1;
            AIA(r,17) = -1;
            AIA(r,18) = -1;
            AIA(r,19) = -1;
        end
    end 
    
    %% Record Agent's Veclocity Values
    VelocityAgentArray(iterations,:) = AIA(:,19)';
    
    %% Controller for enemy agents
    
    for p = (NOD+1):N
        dxi(1:2, p) = EnemyControllerFun(x(1:2,p), target_condition(1:2, p));
    end
    
    
    %% New Controller for Defender Agents
    EnemyArray(1:NOE,1:4) = AIA((NOD+1):N, 1:4);
    EnemyArray(1:NOE,5:10) = AIA((NOD+1):N, 11:16);
    
    q = 1;
    
    lim = NOE;
    while q <= lim
        if EnemyArray(q, 7) == 0 || EnemyArray(q, 2) == 2 %|| EnemyArray(q, 10) ~= 0
            EnemyArray(q, :) = [];
            lim = lim - 1;
            q = q - 1;
        end
        q = q + 1;
    end
    
    sizevar = size(EnemyArray);
    NOV = sizevar(1);
        
    DefenderArray(1:NOD, 1:4) = AIA(1:NOD, 1:4);
    DefenderArray(1:NOD, 5:9) = AIA(1:NOD, 14:18);
    DefenderArray(1:NOD, 10) = AIA(1:NOD, 20); %lock
    
    sizevar = size(DefenderArray);
    NOAD = sizevar(1);
    
    if NOV ~= 0
        
        if exist('iterOnce') == 0
            iterOnce = t + WaitIterationTime;
        end
        
        if NOAD ~= 0 && t >= iterOnce
        
            AssignmentArray = zeros(NOAD+1,NOV+1);
            
            if NOV>2
                %disp('greater');
            end
            
            for w = 2:(NOAD+1)
                GlobalDefenderIndex = DefenderArray((w-1), 1);
                AssignmentArray(w,1) = GlobalDefenderIndex;
                
                if DefenderArray((w-1), 7) == 0
                    for e = 2:(NOV+1)
                        
                        GlobalEnemyIndex = EnemyArray((e-1), 1);
                        if AIA(GlobalEnemyIndex, 16) == 0
                            D_abs = norm(x(1:2,GlobalEnemyIndex));
                            D_rel = norm(x(1:2,GlobalEnemyIndex) - x(1:2,GlobalDefenderIndex));
                            U = Urgency(D_rel,D_abs,lambda,gamma);
                            AssignmentArray(1,e) = GlobalEnemyIndex;
                            AssignmentArray(w,e) = U;
                        else
                            AssignmentArray(:,e) = NaN;
                        end
                    end
                else
                    %%fprintf('Defender %.0f is locked on a different target at iteration %.0f.\n', GlobalDefenderIndex, t); 
                    AssignmentArray(w, 1:(NOV+1)) = NaN;
                end
            end
            
            if NOV == 1
                [Value Index] = max(AssignmentArray(2:(NOAD+1),2));
                if ~isnan(Value)
                    LocalEnemyIndex = 1;
                    LocalDefenderIndex = Index;
                    GlobalEnemyIndex = EnemyArray(LocalEnemyIndex,1);
                    DefenderArray(LocalDefenderIndex, 7) = GlobalEnemyIndex;
                end
            else
                
                AssignmentArray( all( isnan( AssignmentArray ), 2 ), : ) = [];
                AssignmentArray( :, all( isnan( AssignmentArray ), 1 ) ) = [];
                sizevar = size(AssignmentArray);
                LastRow = sizevar(1); 
                AssignmentArray2 = AssignmentArray(2:LastRow,:);
                
                sizevar = size(AssignmentArray2);
                NOAD_prime = sizevar(1);
                
                NOV_prime = sizevar(2);
                v = 1:1:NOAD_prime;
                p = perms(v);
    
                sizevar = size(p);
                NumOfPerm = sizevar(1);
                ScoreTable = [];
                
                for g = 1:NumOfPerm
                    order = p(g, 1:NOAD_prime);
                    AssignmentArray3 = AssignmentArray2;
                    TotalScore = 0;
                    for ww = 1:NOAD_prime
                        RowOrder = order(ww);
                        [UrgencyValue Index] = max(AssignmentArray3(RowOrder,2:NOV_prime));
                        if ~isnan(UrgencyValue)      
                            TotalScore = TotalScore + UrgencyValue;
                            AssignmentArray3(:,(Index+1)) = NaN;
                        end
                    end
                    ScoreTable(g, 1) = g;
                    ScoreTable(g, 2) = TotalScore;
                    
                end
                
                ScoreTable = sortrows(ScoreTable, 2);
                ScoreTable = flipud(ScoreTable);
                
                BestOrder = p(ScoreTable(1,1), 1:NOAD_prime);    
                
                for ww = (2:NOAD_prime+1)
                    RowOrder = BestOrder(ww-1);
                    [UrgencyValue Index] = max(AssignmentArray(RowOrder+1,2:NOV_prime));
                    DefIndex = AssignmentArray(RowOrder+1,1);
                    if ~isnan(UrgencyValue)
                        LocalEnemyIndex = Index+1;
                        GlobalEnemyIndex = AssignmentArray(1,LocalEnemyIndex);
                        DefenderArray(DefIndex, 7) = GlobalEnemyIndex;
                        DefenderArray(DefIndex, 6) = UrgencyValue;
                        AssignmentArray(2:NOAD_prime+1,Index+1) = NaN;
                    else
                        %fprintf('Defender %.0f doesnt have a target to pursue at iteration %.0f.\n', DefenderArray((ww-1),1), t); 
                    end
                end

                
            end
            
            for b = 1:NOAD
                Ene = DefenderArray(b,7);
                Def = DefenderArray(b,1);
                
                if Ene ~= 0 && Ene ~= 0
                    AIA(Ene,16) = Def;
                    if AIA(Def,16)~=Ene
                        AIA(Def,16) = Ene;
                        %fprintf('Defender %.0f started to chase a new enemy agent %.0f at iteration %.2f.\n', Def, Ene, t); 
                        AIA(Def,17) = 1;
                    else
                        %fprintf('Defender %.0f is still chasing the same enemy agent %.0f at iteration %.2f.\n', Def, Ene, t); 
                        AIA(Def,17) = 0;
                    end
                end
            end 
            
        end
    end
    
    
    % Defender Control Mode
    if DefenderController.type == 1 
        CircularRotationRate = DefenderController.DefenderControlRate;
        PatrolRadius = DefenderController.Def_radius_init ;
        
        if CircularRotationRate ~= 0
            if t == start2
                for n = 1:NOD
                    tt = (n*CircularRotationInc)+t;
                    xx = PatrolRadius.*cos(deg2rad(tt));
                    yy = PatrolRadius.*sin(deg2rad(tt));
                    RotationMode(1:2,n) = [xx;yy];
                end
                start2 = start2 + CircularRotationRate;
            end 
           
        end
        
        for n = 1:NOD
            if AIA(n,16) == 0
                dxi(1:2, n) = DefenderControllerFun(x(1:2, n), RotationMode(1:2,n));
            else
                Target_Enemy = AIA(n,16);
                dxi(1:2, n) = DefenderControllerFun(x(1:2, n), x(1:2, Target_Enemy));
            end
        end 
        
    elseif DefenderController.type == 2
        DirectionVec = [DefenderController(1,2),DefenderController(1,3)] ;
        DirectionVec = DirectionVec';
        
        for n = 1:NOD
            if AIA(n,16) == 0
                dxi(1:2, n) = DefenderController(x(1:2, n),DirectionVec);
            else
                Target_Enemy = AIA(n,16);
                dxi(1:2, n) = DefenderController(x(1:2, n),x(1:2, Target_Enemy));
            end
        end
    
    end
    
    %% Capturing Procedure
    row =1;
    for l = 1:NOD
        if AIA(l,16) ~= 0
            CaptureArray(row, 1) = l;
            CaptureArray(row, 2) = AIA(l,16);
            EnemyInd = AIA(l,16);
            DefenderPos = [AIA(l,3),AIA(l,4)];
            EnemyPos = [AIA(EnemyInd,3),AIA(EnemyInd,4)];
            Distance = norm([(DefenderPos(1) - EnemyPos(1)),(DefenderPos(2) - EnemyPos(2))]);
            CaptureArray(row, 3) = Distance;

            if AIA(l,17) == 0
                CaptureArray(row, 4) = AIA(l,18);
            elseif AIA(l,17) == 1
                AIA(l,18) = DefTimer;
                %fprintf('Column 17 value is 1, so timer has restarted for defender %.0f at iteration %.0f! Locking is failed and set to 0. \n',l, t);
                CaptureArray(row, 4) = AIA(l,18);
                AIA(l,17) = 0;
                AIA(l,20) = 0;
                
            end
            row = row + 1;
        end
    end
    
    if (exist('CaptureArray') == 1)
        sizevar = size(CaptureArray);
        CaptureArraySize = sizevar(1); 
        
        if CaptureArraySize > 0 && CaptureArray(1,1) ~= 0
            for y = 1:CaptureArraySize
                DistanceLimit = CaptureArray(y, 3);
                
                if (DistanceLimit < 0.22 ) && ((CaptureArray(y, 4)) >= 2)
                    CaptureArray(y, 4) = (CaptureArray(y, 4) - 1);
                    AIA(CaptureArray(y, 1),18) = CaptureArray(y, 4);
                    AIA(CaptureArray(y, 1),20) = 1; %locking
                    %fprintf('Defender %.0f is Chasing Enemy: %.0f at Timer: %.0f with Distance of %.5f ! \n', CaptureArray(y, 1), CaptureArray(y, 2),CaptureArray(y, 4),  CaptureArray(y, 3));
                
                elseif (DistanceLimit < 0.25) && ((CaptureArray(y, 4)) < 2)
                    EnemyTarget = CaptureArray(y, 2);
                    ra.set_CapturedEnemies_New(EnemyTarget);
                    AIA((CaptureArray(y, 2)), 2) = 2;
                    AIA((CaptureArray(y, 1)),20) = 0;
                    AIA((CaptureArray(y, 1)), 16) = 0;
                    CapturedEnemy = ra.get_captured_enemy();
                    CapturedEnemy_Size = size(CapturedEnemy);
                    TotalCapturedEnemy = CapturedEnemy_Size(1);
                    %fprintf('Enemy %.0f is captured by defender %.0f at iteration %.0f! \n',CaptureArray(y, 2), CaptureArray(y, 1), t);
                    CaptureArray(y,:) = 0;   
                    
                    
                end
            end
            
             CaptureArray( ~any(CaptureArray,2), : ) = [];
            
        end
    end

    
    %% LefT oVERS
    CapturedEnemy = ra.get_captured_enemy();
      
    CapturedEnemy_Size = size(CapturedEnemy);
    TotalCapturedEnemy = CapturedEnemy_Size(1);
    end_clock = clock;
    minutes = end_clock(5) - start_clock(5);
    sec = end_clock(6) - start_clock(6); 
    
    for nn = NOD+1:N
        if norm(x(1:2, nn)) < CompoundRadius && AIA(nn,2) ~= 2
            AIA(nn,21) = 1;
        end
    end

    if (TotalCapturedEnemy) == (N-NOD)
        fprintf('All the enemies are captured!!\n');
        fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
        fprintf('Duration: %.0f iterations! \n', t);
        TargetHit = AIA(NOD+1:N, 21);
        TargetHit = sum(TargetHit(:) == 1);
        fprintf('The target got hit: %.0f ! \n', TargetHit);
        VelocityAgentArray( ~any(VelocityAgentArray,2), : ) = [];
        ra.call_at_scripts_end();
        Return = [];
        Return.VelocityAgentArray = VelocityAgentArray;
        Return.TargetHit = TargetHit;
%         Return.NOV12TimeIncr = NOV12TimeIncr;
        return;
    end
    
    CapturedEnemy_Size = size(CapturedEnemy);
    NumberOfCapturedEnemies = CapturedEnemy_Size(1);
      
    if NumberOfCapturedEnemies == 0
        for z = (NOD+1):N

            AIA(z,5) = x(1,z); % previous x element of position vector of agent z
            AIA(z,6) = x(2,z); % previous y 'element of position vector of agent z
        end 
    
    else
    
        for z = (NOD+1):N
            
            AlreadyCaptured = 0;
            
            for k = 1:NumberOfCapturedEnemies
                if z == CapturedEnemy(k)
                    AlreadyCaptured = 1;
                end
            end

            if AlreadyCaptured == 0
                AIA(z,5) = x(1,z); % previous x element of position vector of agent z
                AIA(z,6) = x(2,z); % previous y element of position vector of agent z
            elseif AlreadyCaptured == 1
                AIA(z,5) = 0;
                AIA(z,6) = 0;
            end
               
        end   
    end
    
    for rr = 1:NOD
        AIA(rr,5) = x(1,rr); % previous x element of position vector of agent r
        AIA(rr,6) = x(2,rr); % previous y element of position vector of agent r
    end
    
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send DefenderInfoocities to agents
    ra.set_velocities(1:N, dxu);
    ra.step();

end

end_clock = clock;
minutes = end_clock(5) - start_clock(5);
sec = end_clock(6) - start_clock(6);

fprintf('%.0f enemies are captured!!\n',size(CapturedEnemy));
fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
fprintf('Duration: %.0f iterations! \n', t);
VelocityAgentArray( ~any(VelocityAgentArray,2), : ) = [];
TargetHit = AIA(NOD+1:N, 21);
TargetHit = sum(TargetHit(:) == 1);
fprintf('The target got hit: %.0f ! \n', TargetHit);
Return = [];
Return.VelocityAgentArray = VelocityAgentArray;
Return.TargetHit = TargetHit;
ra.call_at_scripts_end();
return
 
function U = Urgency(D_rel,D_abs,lambda,gamma)
    
    U = 1 / (((D_rel)^lambda)*((D_abs)^gamma));
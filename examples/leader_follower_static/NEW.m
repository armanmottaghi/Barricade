function Return = Simulation()
start_clock = clock;
fprintf('Start Time: %.2f hours, %.2f minutes and %.2f seconds.\n', start_clock(4), start_clock(5),start_clock(6) );
%Run the simulation for a specific number of iterations
iterations = 20000;
set(handles.checkbox1, 'value', 0);
handles.pause = 0;

%% Set up the Robotarium object

%Get Robotarium object and set the save parameters
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
NOD = str2num(get(handles.edit1,'String'));
NOE = str2num(get(handles.edit2,'String'));

N = NOD + NOE;

handles.N = N;
handles.NOD = NOD;
handles.NOE = NOE;

VelocityAgentArray = zeros(iterations,N);

% NOD = 8;
% NOE = N-NOD;
% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
ra = rb.set_number_of_agents(N);
ra = rb.set_GUIaxes(handles.axes1);
ra = rb.set_save_data(false);
CapturedEnemy = [];
ra = rb.set_CapturedEnemies(CapturedEnemy);
ra = rb.set_number_of_enemies(NOE);
ra = rb.set_number_of_defenders(NOD);
ra = rb.build();


%% Create the desired Laplacian

%Graph laplacian
L_Enemies = DiagLapMatrix(N-1, -1);
L = zeros(N, N);
L(2:N, 2:N) = L_Enemies;
L(1, 1) = 1;

%Initialize DefenderInfoocity vector
dxi = zeros(2, N);

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.0;
CompoundBuildingRadius = str2num(get(handles.edit7,'String'));


%% Grab tools we need to convert from single-integrator to unicycle dynamics
% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 1, 'AngularVelocityLimit', 1);
% Single-integrator barrier certificates
%si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.005);
% Single-integrator position controller
EnemyController = create_si_position_controller('VelocityMagnitudeLimit', 0.046);
DefenderController = create_si_position_controller();
%distance = zeros(iterations, 2);

x = ra.get_poses();

handles.x = x;

target_condition = zeros(3,N);

if get(handles.radio4, 'value') == 1
    for rq = (NOD+1):N
        target_condition(1:2,rq) = (-1 * x(1:2,rq));
        %target_condition(1:2,rq) =[0;0];
        target_condition(3,rq) = x(3,rq);
    end
    
    handles.target_condition = target_condition;
    
elseif get(handles.radio5, 'value') == 1
    for rq = (NOD+1):N
        dirVec = str2num(get(handles.edit6, 'string'));
        dirVec = dirVec';
        target_condition(1:2,rq) = dirVec;
        target_condition(3,rq) = x(3,rq);
    end
    
    handles.target_condition = target_condition;
end

guidata(hObject,handles)

% for r = 2:N
%     target_condition(1:2,r) = (-1 * x(1:2,r));
%     %target_condition(1:2,r) =[0;0];
%     target_condition(3,r) = x(3,r);
% end

% inc	 = (iterations/60);
RotationMode = zeros(2, NOD);
CircularRotationInc = 360/NOD;


start2= 1;
start = 1;
set(handles.text2, 'string', num2str(start));
handles.start = start;
DefTimer = str2num(get(handles.edit8,'String'));
AIA = zeros (N,20);

for tan = 1:N
    AgentsList(tan,1) = tan;
end
set(handles.popupmenu4,'String', num2str(AgentsList));

%figure2 = figure;
%figure2handle = gca;
CaptureArray = zeros(1,4);
for t = 1:iterations
    
    set(handles.text28, 'string', num2str(t));
    handles.iterations = t;
    clock1 = clock;
    set(handles.text29, 'string', num2str(clock1(6)));
    
    %% Generate Random X,Y destination coordinates after every 'inc' intervals
    if get(handles.radio3,'value') == 1
        RandomMoveRate = str2num(get(handles.edit5, 'string'));
        start = str2num(get(handles.text2, 'string'));
        %fprintf('handles.start VALUE: %.0f', handles.start);
        if t == start
            for u = (NOD+1):N
                ap=2*pi*rand;
                rp=sqrt(rand);
                xp=(10*rp)*cos(ap);
                yp=(10*rp)*sin(ap);
                target_condition(1:2,u) = [xp;yp];
            end

            start = start + RandomMoveRate;
            handles.start = start;
            set(handles.text2, 'string', num2str(start));
        end
        
        handles.target_condition = target_condition;
    end
    %% Get Poses
  
    x = ra.get_poses();
    handles.x = x;
    CapturedEnemy = ra.get_captured_enemy();
    guidata(hObject,handles);

    %% Update Popdown Menu
    AgentsList = setdiff(AgentsList,CapturedEnemy);
    AgentsList = num2str(AgentsList);
    set(handles.popupmenu4,'String',AgentsList);
    

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
            
            if r == 2
                set(handles.text30, 'string', num2str(norm([AIA(r,3), AIA(r,4)])));
            end 
            if r == 1
                set(handles.text31, 'string', num2str(norm([AIA(r,3), AIA(r,4)])));
            end       
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
                AIA(r,12) = rad2deg(atan(CompoundBuildingRadius/(norm([AIA(r,7),AIA(r,8)])))); % Critial angle
            else
                AIA(r,12) = 0;
            end

            %Column 13: Determining whether enemy r has violated the safe space or not
            if (norm(x(1:2,r)) < 4) && (AIA(r,2) == 1)
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
    
    VelocityAgentArray(t,:) = AIA(:,19)';
    
    %% Controller for enemy agents
    
    if get(handles.checkbox4,'value') == 1
        guidata(hObject, handles);
        target_condition = handles.target_condition;
        for p = (NOD+1):N
            dxi(1:2, p) = EnemyController(x(1:2,p), target_condition(1:2, p));
            %dxi2(1:2, p) = EnemyController2(x(1:2,p), target_condition(1:2, p));
        end
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
       
    
    %NOV = sum((EnemyArray(:,7))==1); %Number of violaters 
    lambda = 5;
    gamma = 1;
    
    sizevar = size(DefenderArray);
    NOAD = sizevar(1);
    
    if NOV ~= 0
        
        if exist('iterOnce') == 0
            iterOnce = t + 1;
        end
        
        if NOAD ~= 0 && t >= iterOnce
        
            AssignmentArray = zeros(NOAD+1,NOV+1);
            
            if NOV>2
                disp('greater');
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
                            %U=D_rel;
                            AssignmentArray(1,e) = GlobalEnemyIndex;
                            AssignmentArray(w,e) = U;
                        else
                            AssignmentArray(:,e) = NaN;
                        end
                    end
                else
                    %fprintf('Defender %.0f is locked on a different target at iteration %.0f.\n', GlobalDefenderIndex, t); 
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
                        fprintf('Defender %.0f doesnt have a target to pursue at iteration %.0f.\n', DefenderArray((ww-1),1), t); 
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
                        fprintf('Defender %.0f started to chase a new enemy agent %.0f at iteration %.2f.\n', Def, Ene, t); 
                        AIA(Def,17) = 1;
                    else
                        fprintf('Defender %.0f is still chasing the same enemy agent %.0f at iteration %.2f.\n', Def, Ene, t); 
                        AIA(Def,17) = 0;
                    end
                end
            end 
            
        end
    end
    
    
    % Defender Control Mode
    
    if get(handles.radio1, 'value') == 1
        CircularRotationRate = str2num(get(handles.edit9,'String'));
        PatrolRadius = str2num(get(handles.edit3,'String'));
        %disp(PatrolRadius);
        if t == start2
            for n = 1:NOD
                tt = (n*CircularRotationInc)+t;
                xx = PatrolRadius.*cos(tt);
                yy = PatrolRadius.*sin(tt);
                RotationMode(1:2,n) = [xx;yy];
            end
            start2 = start2 + CircularRotationRate;
        end 
        
        for n = 1:NOD
            if AIA(n,16) == 0
%                 dxi(1:2, n) = formation_control_gain*(norm(RotationMode(1:2,n) - x(1:2, n))^2)*(RotationMode(1:2,n) - x(1:2, n));
                dxi(1:2, n) = DefenderController(x(1:2, n), RotationMode(1:2,n));
            else
                Target_Enemy = AIA(n,16);
                dxi(1:2, n) = DefenderController(x(1:2, n), x(1:2, Target_Enemy));
                %dxi(1:2, n) = formation_control_gain*(norm(x(1:2, Target_Enemy) - x(1:2, n))^2 -  desired_distance^2)*(x(1:2, Target_Enemy) - x(1:2, n));
            end
        end 
        
    elseif get(handles.radio2, 'value') == 1
        DirectionVec = str2num(get(handles.edit4,'String'));
        DirectionVec = DirectionVec';
        for n = 1:NOD
            if AIA(n,16) == 0
                %dxi(1:2, n) = formation_control_gain*(norm(DirectionVec - x(1:2, n))^2)*(DirectionVec - x(1:2, n));
                dxi(1:2, n) = DefenderController(x(1:2, n),DirectionVec);
            else
                Target_Enemy = AIA(n,16);
                %dxi(1:2, n) = formation_control_gain*(norm(x(1:2, Target_Enemy) - x(1:2, n))^2 -  desired_distance^2)*(x(1:2, Target_Enemy) - x(1:2, n));
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
                fprintf('Column 17 value is 1, so timer has restarted for defender %.0f at iteration %.0f! Locking is failed and set to 0. \n',l, t);
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
                    fprintf('Defender %.0f is Chasing Enemy: %.0f at Timer: %.0f with Distance of %.5f ! \n', CaptureArray(y, 1), CaptureArray(y, 2),CaptureArray(y, 4),  CaptureArray(y, 3));
                
                elseif (DistanceLimit < 0.25) && ((CaptureArray(y, 4)) < 2)
                    EnemyTarget = CaptureArray(y, 2);
                    ra.set_CapturedEnemies_New(EnemyTarget);
                    AIA((CaptureArray(y, 2)), 2) = 2;
                    AIA((CaptureArray(y, 1)),20) = 0;
                    AIA((CaptureArray(y, 1)), 16) = 0;
                    CapturedEnemy = ra.get_captured_enemy();
                    CapturedEnemy_Size = size(CapturedEnemy);
                    TotalCapturedEnemy = CapturedEnemy_Size(1);
                    fprintf('Enemy %.0f is captured by defender %.0f at iteration %.0f! \n',CaptureArray(y, 2), CaptureArray(y, 1), t);
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
    
    guidata(hObject, handles);
    
    StopStatus = get(handles.checkbox1, 'value');
    
    if StopStatus == 1
        fprintf('Simulation is stopped!\n');
        fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
        fprintf('Duration: %.0f iterations! \n', t);
        fprintf('Enemies captured: %.0f \n', TotalCapturedEnemy);
        VelocityAgentArray( ~any(VelocityAgentArray,2), : ) = [];
        ra.call_at_scripts_end();
        break;
        
    end
    
    if (TotalCapturedEnemy) == (N-NOD)
        
        fprintf('All the enemies are captured!!\n');
        fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
        fprintf('Duration: %.0f iterations! \n', t);
        pushbutton3_Callback(hObject, eventdata, handles);
        msgbox('DONE');
        VelocityAgentArray( ~any(VelocityAgentArray,2), : ) = [];
        boxplot(VelocityAgentArray(:,1:NOD),1:1:NOD)
        title('Defender Velocity Box Plot')
        xlabel('Defender')
        ylabel('Velocity')
        ra.call_at_scripts_end();
        break;
        
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
    %dxi = si_barrier_cert(dxi, x);
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send DefenderInfoocities to agents
    %Set DefenderInfoocities
    ra.set_velocities(1:N, dxu);
    ra.step();
    guidata(hObject, handles);
end

end_clock = clock;
minutes = end_clock(5) - start_clock(5);
sec = end_clock(6) - start_clock(6);

fprintf('%.0f enemies are captured!!\n',size(CapturedEnemy));
fprintf('Duration: %.0f min and %.5f seconds!\n ', minutes , sec);
fprintf('Duration: %.0f iterations! \n', t);
VelocityAgentArray( ~any(VelocityAgentArray,2), : ) = [];
ra.call_at_scripts_end();

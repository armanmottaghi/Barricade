function Main()

%% Initialization
EnemyControlRate = 0; 
DefenderControlRate = 15;
iterations = 700;
BoundaryLength = 1000;
EnemyVelocityLimit = 16;
DefenderVelocityLimit = 24;
DefenderController.type = 1;
Def_radius_init = 400;
gamma =2;
lambda = 5;
DefTimer = 10;
WaitIterationTime = 1;
XVelocityGain = 1;
YVelocityGain = 1;
EnemyController.type = 2;
Enemy_radius_ini = 800;
NOD = 4;
NOE = 10;
WatchZoneRadius = 500;
CompoundRadius = 150;
TimeStep =1 ;

%% Assigning
l = BoundaryLength;
boundaries = [-l, l, -l, l];
boundary_points = {[-l, l, l, -l], [-l, -l, l, l]};
DefenderController.Def_radius_init = Def_radius_init;
DefenderController.DefenderControlRate = DefenderControlRate; %Def_radius: initial start point
DefenderController.XVelocityGain = XVelocityGain;
DefenderController.YVelocityGain = YVelocityGain;
DefenderController.gamma = gamma;
DefenderController.lambda = lambda;
DefenderController.DefTimer = DefTimer;
DefenderController.DefenderVelocityLimit = DefenderVelocityLimit;
EnemyController.Enemy_radius_ini = Enemy_radius_ini;
EnemyController.EnemyControlRate = EnemyControlRate;
EnemyController.EnemyVelocityLimit = EnemyVelocityLimit;
EnvironmentalParameters.CompoundRadius = CompoundRadius;
EnvironmentalParameters.WatchZoneRadius = WatchZoneRadius;
EnvironmentalParameters.TimeStep = TimeStep;
EnvironmentalParameters.WaitIterationTime = WaitIterationTime;
EnvironmentalParameters.NOD = NOD;
EnvironmentalParameters.NOE = NOE;
EnvironmentalParameters.boundaries = boundaries;
EnvironmentalParameters.boundary_points = boundary_points;
EnvironmentalParameters.iterations = iterations;

%% Running
Return = Simulation(DefenderController,EnemyController,EnvironmentalParameters);

%% Output
VelocityAgentArray = Return.VelocityAgentArray;
TargetHit = Return.TargetHit;
fprintf('Hit %.0f ! for k: %.0f!',TargetHit);
Time = clock;
filename = sprintf('Sim %d',Time(6));
csvwrite(filename,VelocityAgentArray)
end


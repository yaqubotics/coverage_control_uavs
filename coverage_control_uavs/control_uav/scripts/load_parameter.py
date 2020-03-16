import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import compute_coverage as com
import sys
import os, json
def TrueFunction(par):
	TF = np.zeros((len(par.Xaxis),len(par.Yaxis)))
	for h in xrange(len(par.Xaxis)):
		for k in xrange(len(par.Yaxis)):
			TF[k,h] = com.computeSensoryFunction(par.Xaxis[h],par.Yaxis[k],par)
	
	#print TF
	return TF
	
class Parameter(object):
	dir_path = os.path.dirname(os.path.realpath(__file__))
	with open(dir_path+'/config.json','r') as f:
		config = json.load(f)
	# parameters to specify the grid of points 
	#(this is the fixed and more dense grid used for the Voronoi regions)
	GridStart = config["DEFAULT"]["GRID_START"]      
	GridEnd = config["DEFAULT"]["GRID_END"]
	GridStep = config["DEFAULT"]["GRID_STEP"]
	
	RealScale = config["DEFAULT"]["REAL_SCALE"]
	HalfRealScale = RealScale/2
	FinerGridStep = config["DEFAULT"]["FINER_GRID_STEP"]
	StaticAltitude = config["DEFAULT"]["STATIC_ALTITUDE"]
	# valuse of the Regularization Parameter
	RegularizationConstant = config["DEFAULT"]["REGULARIZATION_CONSTANT"]
	
	KernelStd = 5*2*GridStep
	
	KernelType = 'gaussian'
	
	# number of iterations
	Niter = config["DEFAULT"]["NUM_ITER"]
	IterationNumberList = np.ones((Niter,1))
	for i in range(Niter-1):
		IterationNumberList[i,0]=i
	# cost with respect to centroids or positions
	CostFlag = 'centroid'
	
	# measurement noise standard deviation
	NoiseStd = RegularizationConstant
	
	## Parameters for the control input        
	
	# Variance Exponential. If it is >1 we prefer exploitation if it is 
	# 0<x<1 we prefer exploration. 
	VarianceExponentialShaping = config["DEFAULT"]["VARIANCE_EXPONENTIAL_SHAPING"]
	TradeOffCoeff = 0.3 # 0-1 # tidak dipakai
	# This parameter here MUST be always equal to Positions, change it just
	# after the partition initialization
	VoronoiMode = 'Positions'
	
	## creation of the grid structure for the voronoi evaluation
	# values of the x axis
	Xaxis = np.arange(GridStart,GridEnd+GridStep,GridStep).reshape(1,int((GridEnd-GridStart)/GridStep+1))
	Xaxis = np.transpose(Xaxis)
	# values of the y axis
	Yaxis = Xaxis
	# number of pionts in the grid
	NumberOfPoints = len(Xaxis)
	
	# creation of the matrix of point to manage the mesh plot
	Xgrid,Ygrid = np.meshgrid(Xaxis,Yaxis)
	
	# area of a single unitary square in the grid
	UnitArea = GridStep*GridStep
	
	CentroidNonConvex = 'MinCost' # 'Median' or 'Weighting' or 'MinCost'
	Estimator = 'Gaussian' # 'Gaussian' or 'Kriging' 
	#x = Parameter()	
	#TrueFunction = TrueFun(x)
	## Initial positions
	#agent_pos_x = np.matrix('0.15 0.20 0.35 0.25 0.50 0.65 0.70 0.85')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.90 0.70 0.15 0.60 0.85 0.05 0.40 0.15')#.reshape(1,NumberOfAgents)
	#agent_pos_x = np.matrix('0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.80')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.10 0.10 0.10 0.10 0.10 0.10 0.10 0.10')#.reshape(1,NumberOfAgents)
	# number of agents
	NumberOfAgents = len(config["UAV"])
	agent_pos_x = np.zeros((1,NumberOfAgents))
	agent_pos_y = np.zeros((1,NumberOfAgents))
	for i in range(NumberOfAgents):
		agent_pos_x[0][i]=float(config["UAV"][repr(i+1)]["DEST_POS"][0])/RealScale
		agent_pos_y[0][i]=float(config["UAV"][repr(i+1)]["DEST_POS"][1])/RealScale
	
	all_init_pos = []
	for i in range(NumberOfAgents):
		all_init_pos.append(config["UAV"][repr(i+1)]["INIT_POS"]) 
	#NumberOfAgents = 8
	#agent_pos_x = np.matrix('0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.14 0.10 0.10 0.10 0.10')#.reshape(1,NumberOfAgents)
	#NumberOfAgents = 4
	#agent_pos_x = np.matrix('0.0 0.0 1.0 1.0')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.0 1.0 0.0 1.0')#.reshape(1,NumberOfAgents)
	#NumberOfAgents = 3
	#agent_pos_x = np.matrix('0.46 0.54 0.60')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14')#.reshape(1,NumberOfAgents)

	#NumberOfAgents = 6
	#agent_pos_x = np.matrix('0.46 0.54 0.62 0.46 0.54 0.62')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.06 0.06 0.06')#.reshape(1,NumberOfAgents)

	#NumberOfAgents = 9
	#agent_pos_x = np.matrix('0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64 0.72')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.14 0.10 0.10 0.10 0.10 0.10')#.reshape(1,NumberOfAgents)
	#NumberOfAgents = 8
	#agent_pos_x = np.matrix('0.44 0.44 0.44 0.50 0.50 0.56 0.56 0.56')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.44 0.50 0.56 0.44 0.56 0.44 0.50 0.56')#.reshape(1,NumberOfAgents)
	#NumberOfAgents = 12
	#agent_pos_x = np.matrix('0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64 0.72 0.40 0.46 0.54')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.14 0.10 0.10 0.10 0.10 0.10 0.06 0.06 0.06')#.reshape(1,NumberOfAgents)

	#NumberOfAgents = 7
	#agent_pos_x = np.matrix('0.46 0.54 0.62 0.46 0.54 0.62 0.70')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.06 0.06 0.06 0.06')#.reshape(1,NumberOfAgents)

	#NumberOfAgents = 16
	#agent_pos_x = np.matrix('0.34 0.32 0.38 0.34 0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64 0.72 0.40 0.46 0.54')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.06 0.10 0.10 0.14 0.14 0.14 0.14 0.14 0.10 0.10 0.10 0.10 0.10 0.06 0.06 0.06')#.reshape(1,NumberOfAgents)

	#NumberOfAgents = 24
	#agent_pos_x = np.matrix('0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64 0.72 0.40 0.46 0.54 0.40 0.46 0.54 0.60 0.44 0.50 0.58 0.64 0.38 0.40 0.46 0.54')#.reshape(1,NumberOfAgents)
	#agent_pos_y = np.matrix('0.14 0.14 0.14 0.14 0.10 0.10 0.10 0.10 0.10 0.06 0.06 0.06 0.98 0.98 0.98 0.98 0.94 0.94 0.94 0.94 0.94 0.90 0.90 0.90')#.reshape(1,NumberOfAgents)
	Positions = np.append(np.transpose(agent_pos_x), np.transpose(agent_pos_y), axis=1) #0.5*rand(Parameter.NumberOfAgents,2);

	Altitude = 0.04

	## motion constraints
	MotionConstraintsThreshold = np.inf*GridEnd;
	UsingObstacle = config["DEFAULT"]["USING_OBSTACLE"] #True
	TimeoutHRVO = config["DEFAULT"]["TIMEOUT_HRVO"]
	UsingGlobalPlanner = config["DEFAULT"]["USING_GLOBAL_PLANNER"] #True
	UsingAnytimeAStar = False #True
	AnytimeAStarSampling = 1.0
	UsingVarianceAsAStarCost = False #True #True
	UsingNeighborPathAsCost = False
	UsingGaussianCost = False
	VarianceCostWeight = 1.5
	UniversalCost = 30.0
	RearrangeGoalPosition = False
	ModifiedVoronoi = False
	ConsideringCoincidedGoal = config["DEFAULT"]["CONSIDERING_COINCIDED_GOAL"]
	V_MAX = config["DEFAULT"]["V_MAX"]

	UsingStrategy = config["DEFAULT"]["USING_STRATEGY"] #1: default, 2 : direct (wait only 1/2 num of agent)
	CurrentIteration = 0
	stopWithin = 'Iteration' #  'MaxVariance' # 
	stopMaxVariance = config["DEFAULT"]["STOP_MAX_VARIANCE"]
	#------------------------------
	#define workspace model
	ws_model = dict()
	#robot radius
	ws_model['robot_radius'] = config["DEFAULT"]["ROBOT_RADIUS"]
	ws_model['robot_clearence'] = config["DEFAULT"]["ROBOT_CLEARENCE"]
	#circular obstacles, format [x,y,rad]
	# no obstacles
	#ws_model['circular_obstacles'] = []
	num_of_obstacles = len(config["OBSTACLE"])
	ws_model['circular_obstacles'] = []
	ws_model['rectangle_obstacles'] = []
	if(UsingObstacle):
		for i in range(num_of_obstacles):
			if(config["OBSTACLE"][repr(i)]["TYPE"] == "CYLINDER"):
				obstacle_pose = config["OBSTACLE"][repr(i)]["POSE"]
				obstacle_size = config["OBSTACLE"][repr(i)]["SIZE"]
				ws_model['circular_obstacles'].append([obstacle_pose[0],obstacle_pose[1],obstacle_size[0]])
			elif(config["OBSTACLE"][repr(i)]["TYPE"] == "BOX"):
				obstacle_pose = config["OBSTACLE"][repr(i)]["POSE"]
				obstacle_size = config["OBSTACLE"][repr(i)]["SIZE"]
				ws_model['rectangle_obstacles'].append([obstacle_pose[0],obstacle_pose[1],obstacle_size[0],obstacle_size[1]])
			else:
				print "obstacle id = "+repr(i)+" type = "+config["OBSTACLE"][repr(i)]["TYPE"]+"is not recognized"
			
	'''
	if(UsingObstacle):
		# with obstacles
		#ws_model['circular_obstacles'] = [[0.2, 0.2, 0.05], [0.35, 0.8, 0.05], [0.8, 0.2, 0.05], [0.65, 0.8, 0.05], [0.5, 0.5, 0.03], [0.9,0.6,0.02],[0.1,0.6,0.02], [0.5,0.3,0.02]]
		ws_model['circular_obstacles'] = [[0.2, 0.5, 0.2], [0.8, 0.2, 0.05], [0.85, 0.8, 0.15]]
		#ws_model['circular_obstacles'] = [[0.2, 0.2, 0.12], [0.8, 0.8, 0.12], [0.8,0.2,0.05], [0.2,0.8,0.05]]
		#ws_model['circular_obstacles'] = [[0.2, 0.2, 0.03], [0.8, 0.8, 0.03], [0.8,0.2,0.03], [0.2,0.8,0.03]]
		#rectangular boundary, format [x,y,width/2,heigth/2]
	 	ws_model['type_obstacle'] = ['s','s','s','s','s','s','s','s','s'] # s:static, d:dynamic
	 	ws_model['rectangle_obstacles'] = []#[[0.2,0.2, 0.4,0.2],[0.2,0.7,0.1,0.4]]
	else:
	 	ws_model['circular_obstacles'] = []
	 	ws_model['type_obstacle'] = []
	 	ws_model['rectangle_obstacles'] = []
	'''
	ws_model['boundary'] = [-0.1,-0.1,1,1]#[[-0.1,-0.5],[0.5,-0.1],[1.1,0.5],[0.5,1.1]]

	#compute all boundary points of obstacle
	grid_sampling = 0.01
	obs_point = []
	#hole = [px,py,lx,ly]
	for hole in ws_model['rectangle_obstacles']:
		px = hole[0]
		py = hole[1]
		lx = hole[2]
		ly = hole[3]
		for i in range(int(np.abs(lx-px)/grid_sampling)):
			obs_point.append([px+i*grid_sampling,py])
			obs_point.append([px+i*grid_sampling,py+ly])
		for j in range(int(np.abs(ly-py)/grid_sampling)):
			obs_point.append([py+j*grid_sampling,px])
			obs_point.append([py+j*grid_sampling,px+lx])

	ws_model['obs_point'] = obs_point
	ws_model['map_obs'] = []
	ws_model['n_points'] = NumberOfPoints
	ws_model['method'] = config["DEFAULT"]["PATH_PLANNING_METHOD"] #HRVO VO RVO
	ObstacleRegion = np.ones((NumberOfPoints, NumberOfPoints))
	ObstacleRegionWithClearence = np.ones((NumberOfPoints, NumberOfPoints))
	ax=Xaxis
	ay=Yaxis
	ws_model['obs_clearence'] = GridStep
	for hole in ws_model['circular_obstacles']:
		for i in range(NumberOfPoints):
			for j in range(NumberOfPoints):
				dist = np.sqrt((ax[i]-hole[0])*(ax[i]-hole[0])+(ay[j]-hole[1])*(ay[j]-hole[1]))
				if (dist < (1.2*hole[2]+0.01)):
					ObstacleRegion[j,i]=0
				if (dist < (1.2*hole[2]+ws_model['obs_clearence']+ws_model['robot_radius']/RealScale/2)):
					ObstacleRegionWithClearence[j,i]=0
				#if ((ax[i] > hole[0]-hole[2]) and (ax[i] < hole[0]+hole[2]) and (ay[j] > hole[1]-hole[2]) and (ay[j] < hole[1]+hole[2])):
				#	ObstacleRegion[j,i] = 0

	'''
	for k in ws_model['rectangle_obstacles']:
		mapobs = np.ones((NumberOfPoints,NumberOfPoints))
		for i in range(NumberOfPoints):
			for j in range(NumberOfPoints):
				if((Xaxis[i] > k[0]) and (Xaxis[i] < k[0]+k[2]) and (Yaxis[j] > k[1]) and (Yaxis[j] < k[1]+k[3])):
					mapobs[i,j] = 0
		ws_model['map_obs'].append(mapobs)
	np.savetxt('mapobs.out',mapobs,fmt='%i')
	'''
	#print obs_point
	#sys.exit()
'''
par = Parameter()
fig = plt.figure()
ax = fig.gca(projection='3d')
print par.Xgrid.shape
print par.Ygrid.shape
print par.TrueFunction().shape
# Plot the surface.
surf = ax.plot_surface(par.Xgrid, par.Ygrid, par.TrueFunction(), cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
'''


from global_planning import *
import compute_coverage as com
import time
import load_parameter as lp
import sys
from math import *

def Astar_version1(Parameter,start_pos,goal_pos,diagram4):
	tStartGlobalPlanning = time.time()
	
	#print Parameter.ws_model['circular_obstacles']
	agentFlagMayObstruct = False
	print Parameter.GridEnd
	p = (start_pos[0],start_pos[1])
	q = (goal_pos[0],goal_pos[1])
	p=com.highsaturatetuple(p,Parameter.GridEnd)	
	p=com.lowsaturatetuple(p,Parameter.GridStart)
	q=com.highsaturatetuple(q,Parameter.GridEnd)	
	q=com.lowsaturatetuple(q,Parameter.GridStart)
	print Parameter.ws_model
	if not isOutsideAllCircle(Parameter.ws_model,p,q):
		print 'true'
		agentFlagMayObstruct=True

	#sys.exit()
	AllPath = []
	
	if(agentFlagMayObstruct):
		startPlanx = int(floor(p[0]*Parameter.NumberOfPoints))
		startPlany = int(floor(p[1]*Parameter.NumberOfPoints))
		startPlanx,startPlany=com.highsaturate(startPlanx,startPlany,Parameter.NumberOfPoints)
		if(Parameter.ObstacleRegion[startPlany,startPlanx]==0):
			startPlanx = int(ceil(p[0]*Parameter.NumberOfPoints))
			startPlany = int(floor(p[1]*Parameter.NumberOfPoints))
			startPlanx,startPlany=com.highsaturate(startPlanx,startPlany,Parameter.NumberOfPoints)
			if(Parameter.ObstacleRegion[startPlany,startPlanx]==0):
				startPlanx = int(floor(p[0]*Parameter.NumberOfPoints))
				startPlany = int(ceil(p[1]*Parameter.NumberOfPoints))
				startPlanx,startPlany=com.highsaturate(startPlanx,startPlany,Parameter.NumberOfPoints)
				if(Parameter.ObstacleRegion[startPlany,startPlanx]==0):
					startPlanx = int(ceil(p[0]*Parameter.NumberOfPoints))
					startPlany = int(ceil(p[1]*Parameter.NumberOfPoints))
		startPlanx,startPlany=com.highsaturate(startPlanx,startPlany,Parameter.NumberOfPoints)
		startPlanx,startPlany=com.lowsaturate(startPlanx,startPlany,0)
		goalPlanx = int(floor(q[0]*Parameter.NumberOfPoints))
		goalPlany = int(floor(q[1]*Parameter.NumberOfPoints))
		goalPlanx,goalPlany=com.highsaturate(goalPlanx,goalPlany,Parameter.NumberOfPoints)
		if(Parameter.ObstacleRegion[goalPlany,goalPlanx]==0):
			goalPlanx = int(ceil(q[0]*Parameter.NumberOfPoints))
			goalPlany = int(floor(q[1]*Parameter.NumberOfPoints))
			goalPlanx,goalPlany=com.highsaturate(goalPlanx,goalPlany,Parameter.NumberOfPoints)
			if(Parameter.ObstacleRegion[goalPlany,goalPlanx]==0):
				goalPlanx = int(floor(q[0]*Parameter.NumberOfPoints))
				goalPlany = int(ceil(q[1]*Parameter.NumberOfPoints))
				goalPlanx,goalPlany=com.highsaturate(goalPlanx,goalPlany,Parameter.NumberOfPoints)
				if(Parameter.ObstacleRegion[goalPlany,goalPlanx]==0):
					goalPlanx = int(ceil(q[0]*Parameter.NumberOfPoints))
					goalPlany = int(ceil(q[1]*Parameter.NumberOfPoints))
		goalPlanx,goalPlany=com.highsaturate(goalPlanx,goalPlany,Parameter.NumberOfPoints)
		goalPlanx,goalPlany=com.lowsaturate(goalPlanx,goalPlany,0)
		startPlan = (startPlanx, startPlany)
		goalPlan = (goalPlanx,goalPlany)
	else:
		startPlan = (p[0],p[1])
		goalPlan  = (q[0],q[1])


	if (agentFlagMayObstruct):
		#came_from, cost_so_far, priority, cost, heu = a_star_search(diagram4, startPlan, goalPlan)
		came_from, cost_so_far = theta_star_search(diagram4, startPlan, goalPlan)
		try:
			path=reconstruct_path(came_from, start=startPlan, goal=goalPlan)
			#temppath = path[:]
			#path = [temppath[len(temppath)//4],temppath[len(temppath)//2],temppath[3*len(temppath)//4],temppath[len(temppath)-1]]
		except:
			print 'error'
			print 'start at',startPlan,'goal at',goalPlan
			#print diagram4.walls
			#playsound('quite-impressed.mp3')
			path=[(q[0],q[1])]
			agentFlagMayObstruct = False
			#sys.exit()
	else:
		path=[goalPlan]
	
	pathall_x = []
	pathall_y = []
	listTimeToUpdatePath = []
	#for i in range(int(total_time // Parameter.AnytimeAStarSampling )):
	#	listTimeToUpdatePath.append((i+1)*int(total_time // step +1) // total_time // Parameter.AnytimeAStarSampling)

	if (agentFlagMayObstruct):
		for j in range(len(path)):
			pathall_x.append(Parameter.Xaxis[path[j][0]][0])
			pathall_y.append(Parameter.Yaxis[path[j][1]][0])
	else:
		for j in range(len(path)):
			pathall_x.append(path[j][0])
			pathall_y.append(path[j][1])

	tSpentGlobalPlanning = time.time() - tStartGlobalPlanning
	#listTimeGlobalPlanning.append(tSpentGlobalPlanning)	
	return pathall_x,pathall_y
'''
Parameter = lp.Parameter
diagram4 = GridWithWeights(Parameter.NumberOfPoints,Parameter.NumberOfPoints)
diagram4.walls = []
for i in range(Parameter.NumberOfPoints): # berikan wall pada algoritma pathfinding A*
	for j in range(Parameter.NumberOfPoints):
		if (Parameter.ObstacleRegion[j,i]==0):
			diagram4.walls.append((i,j))
for i in range(Parameter.NumberOfPoints):
	for j in range(Parameter.NumberOfPoints):
		if(Parameter.ObstacleRegionWithClearence[j][i] == 0):
			diagram4.weights.update({(i,j): Parameter.UniversalCost})
		else:
			if(Parameter.UsingVarianceAsAStarCost):
				diagram4.weights.update({(i,j): (1.0-RecursiveEstimation.APosterioriVariance[j,i])*Parameter.VarianceCostWeight+graph.cost((0,0), (i,j))}) 



a,b=Astar_version1(Parameter,(0.0,0.0),(0.6,0.6),diagram4)
print a
print b
'''
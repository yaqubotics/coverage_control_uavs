import numpy as np
from collections import namedtuple
from recordtype import recordtype
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import random as rand
import sys
import time
import ctypes as ct
import copy
from multiprocessing import Process
import os

from pykrige.ok import OrdinaryKriging
import pykrige.kriging_tools as kt
from pykrige.uk import UniversalKriging

import matplotlib
import numpy as np
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib.axes as ax
import matplotlib.patches as patches
import matplotlib.artist as artist
def computeCentralizedEstimation(RecursiveEstimation,Parameter):
										
	# special update for the first iteration
	if(Parameter.Estimator == 'Kriging'):
		np.savetxt('debug.out',RecursiveEstimation.InputLocations)
		x = []
		y = []
		phi = []
		N = RecursiveEstimation.InputLocations.shape[0]
		offset = 0.01
		for i in range(N):
			same = False
			for p in range(len(x)):
				for q in range(len(y)):
					if((x[p]+offset > RecursiveEstimation.InputLocations[N-i-1,0]) and (x[p]-offset < RecursiveEstimation.InputLocations[N-i-1,0]) and (y[p]+offset > RecursiveEstimation.InputLocations[N-i-1,1]) and (y[p]-offset < RecursiveEstimation.InputLocations[N-i-1,1])):
						same = True
						break
				if (same == True):
					break
			if(same == False):
				x.append(RecursiveEstimation.InputLocations[N-i-1,0])
				y.append(RecursiveEstimation.InputLocations[N-i-1,1])
				phi.append(RecursiveEstimation.InputLocations[N-i-1,2])
			else:
				print 'There is same point'

		UK = UniversalKriging(x, y, phi, variogram_model='gaussian',
                     verbose=False, enable_plotting=False)
		z, ss = UK.execute('grid', Parameter.Xaxis, Parameter.Yaxis)
		RecursiveEstimation.DensityFunction = z
		RecursiveEstimation.APosterioriVariance = ss 

		RecursiveEstimation.maxAPosterioriVariance.append(np.max(np.max(RecursiveEstimation.APosterioriVariance)))
		RecursiveEstimation.minAPosterioriVariance.append(np.min(np.min(RecursiveEstimation.APosterioriVariance)))
		RecursiveEstimation.averageAPosterioriVariance.append(np.mean(np.mean(RecursiveEstimation.APosterioriVariance)))
	else: # Gaussian Regression
		if (Parameter.CurrentIteration == 1):
			RecursiveEstimation = computeInitialCentralizedEstimation(RecursiveEstimation,Parameter)
			RecursiveEstimation.maxAPosterioriVariance.append(np.max(np.max(RecursiveEstimation.APosterioriVariance)))
			RecursiveEstimation.minAPosterioriVariance.append(np.min(np.min(RecursiveEstimation.APosterioriVariance)))
			RecursiveEstimation.averageAPosterioriVariance.append(np.mean(np.mean(RecursiveEstimation.APosterioriVariance)))
				
			return RecursiveEstimation
		
		NumberOfLocations = RecursiveEstimation.InputLocations.shape[0]
		OldNumberOfLocations = NumberOfLocations - Parameter.NumberOfAgents

		NewSampledKernel = np.zeros((NumberOfLocations,NumberOfLocations))
		NewSampledKernel[0:OldNumberOfLocations,0:OldNumberOfLocations] = RecursiveEstimation.SampledKernel
		RecursiveEstimation.SampledKernel = NewSampledKernel

		for i in range(Parameter.NumberOfAgents):

			SampledKernelColumn = np.exp( -(np.power((RecursiveEstimation.InputLocations[0:OldNumberOfLocations+i+1,0] - \
    	                              RecursiveEstimation.InputLocations[OldNumberOfLocations+i,0]),2) + \
    	                             np.power((RecursiveEstimation.InputLocations[0:OldNumberOfLocations+i+1,1] - \
    	                              RecursiveEstimation.InputLocations[OldNumberOfLocations+i,1]),2) ) \
    	                             / (Parameter.KernelStd**2))

			SampledKernelColumn = np.transpose(SampledKernelColumn)

			RecursiveEstimation.SampledKernel[0:OldNumberOfLocations+i+1,OldNumberOfLocations+i] = SampledKernelColumn
			RecursiveEstimation.SampledKernel[OldNumberOfLocations+i,0:OldNumberOfLocations+i+1] = SampledKernelColumn[0:RecursiveEstimation.SampledKernel.shape[1]];
		end = RecursiveEstimation.SampledKernel.shape[1]
		RecursiveEstimation.InvertedSampledKernel = generalSchurInverse(RecursiveEstimation.InvertedSampledKernel, \
    	                                                            RecursiveEstimation.SampledKernel[0:end-Parameter.NumberOfAgents,end-Parameter.NumberOfAgents:end], \
    	                                                            RecursiveEstimation.SampledKernel[end-Parameter.NumberOfAgents:end,end-Parameter.NumberOfAgents:end]+ \
    	                                                            Parameter.RegularizationConstant*(np.eye(Parameter.NumberOfAgents)))
		
		
		Autocovariance = RecursiveEstimation.InvertedSampledKernel  
		EstimationCoefficients = np.dot(Autocovariance,RecursiveEstimation.InputLocations[:,2].reshape(RecursiveEstimation.InputLocations.shape[0],1))
		#print '(Kbar) dim : ',repr(Autocovariance.shape)
		#print '(y) dim : ',repr(RecursiveEstimation.InputLocations[:,2].reshape(RecursiveEstimation.InputLocations.shape[0],1).shape)
		#print '(c) dim : ',repr(EstimationCoefficients.shape)
		start = time.time()
		Kbar = kernelEvaluation(Parameter.Xaxis,Parameter.Yaxis,RecursiveEstimation.InputLocations, Parameter.KernelStd,RecursiveEstimation.lastKbar,Parameter.NumberOfAgents)
		RecursiveEstimation.lastKbar = Kbar
		#print '(K) dim : ',repr(Kbar.shape)
		temp = np.dot(np.transpose(Kbar),EstimationCoefficients)
		RecursiveEstimation.DensityFunction = np.transpose(temp.reshape(Parameter.NumberOfPoints,Parameter.NumberOfPoints))
		#print "(phi) dim : ",repr(RecursiveEstimation.DensityFunction.shape)
		RecursiveEstimation.APosterioriVariance = np.ones((Parameter.NumberOfPoints,Parameter.NumberOfPoints)) - \
    	        np.transpose(sum(np.multiply(Kbar,np.dot(Autocovariance,Kbar))).reshape(Parameter.NumberOfPoints,Parameter.NumberOfPoints))
		RecursiveEstimation.APosterioriVariancePure=RecursiveEstimation.APosterioriVariance[:]
		RecursiveEstimation.APosterioriVariance=np.multiply(Parameter.ObstacleRegionWithClearence,RecursiveEstimation.APosterioriVariance)

		RecursiveEstimation.maxAPosterioriVariance.append(np.max(np.max(RecursiveEstimation.APosterioriVariance)))
		RecursiveEstimation.minAPosterioriVariance.append(np.min(np.min(RecursiveEstimation.APosterioriVariance)))
		RecursiveEstimation.averageAPosterioriVariance.append(np.mean(np.mean(RecursiveEstimation.APosterioriVariance)))

	return RecursiveEstimation
	
def computeCost(LocalParameter,Node,DensityFunction,flag='centroid'):

	Normalizer = sum(sum(DensityFunction))*LocalParameter.UnitArea
	H = 0
	if(flag == 'position'):
		for i in range(LocalParameter.NumberOfAgents):
			H=(H+sum(sum(np.multiply(np.sqrt( 
					np.power((Node[i].pos[0,0]*np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))-LocalParameter.Xgrid),2)+
					np.power((Node[i].pos[1,0]*np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))-LocalParameter.Ygrid),2)),
					DensityFunction,Node[i].VoronoiRegionWithoutObstacle)/Normalizer))*LocalParameter.UnitArea)
					
	else:
		for i in range(LocalParameter.NumberOfAgents):
			H=(H+sum(sum(np.multiply(np.multiply(np.sqrt( 
					np.power((Node[i].C[0]*np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))-LocalParameter.Xgrid),2)+
					np.power((Node[i].C[1]*np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))-LocalParameter.Ygrid),2)),
					DensityFunction),Node[i].VoronoiRegionWithoutObstacle)/Normalizer))*LocalParameter.UnitArea)			
	return H

def computeCoverage(EstimationStructure,Parameter,Variance,ws_model):
	# compute Voronoi Regions
	#for i in range(Parameter.NumberOfAgents):
	#	print EstimationStructure.Node[i].pos
	EstimationStructure.Node = computeVoronoiRegion(EstimationStructure.Node,Parameter,ws_model)

	EstimationStructure.Node = computeCentroid(EstimationStructure.Node,Parameter,EstimationStructure.DensityFunction,ws_model)
	
	if(not Parameter.ConsideringCoincidedGoal):
		MaxLocations,MaxLocationsValue,EstimationStructure.maxAPosterioriVarianceVoronoi,EstimationStructure.listMaxAPosterioriVarianceVoronoi = computeMaxLocation(EstimationStructure.Node,Parameter,Variance)
		EstimationStructure = computeUpdate(EstimationStructure,Parameter,MaxLocations,MaxLocationsValue,Variance)
	else:
		EstimationStructure = computeMaxLocationModified(EstimationStructure,EstimationStructure.Node,Parameter,Variance)
	return EstimationStructure

def computeClassicCoverage(LocalParameter,NoEstimation,TrueFunction):
	centroid = np.zeros((LocalParameter.NumberOfAgents,2))
	
	NoEstimation.CostFunction.append(computeCost(LocalParameter,NoEstimation.Node,TrueFunction,LocalParameter.CostFlag))
	for t in range(1,LocalParameter.Niter):
		NoEstimation.Node = computeVoronoiRegion(NoEstimation.Node,LocalParameter,LocalParameter.ws_model)
		NoEstimation.Node = computeCentroid(NoEstimation.Node,LocalParameter,TrueFunction,LocalParameter.ws_model)
		for i in range (LocalParameter.NumberOfAgents):
			centroid[i,:] = NoEstimation.Node[i].C
		avEnergy = 0
		for i in range (LocalParameter.NumberOfAgents):
			
			avEnergy = ((avEnergy +
						np.linalg.norm(centroid[i,:] -NoEstimation.Node[i].pos[0])
						/LocalParameter.NumberOfAgents))
			NoEstimation.Node[i].pos = centroid[i,:]
			support = centroid
			
			support=np.delete(support,(i),axis=0)
			NoEstimation.Node[i].pvicini = support         
		NoEstimation.averageEnergy.append(avEnergy)
		for i in range(LocalParameter.NumberOfAgents):
			
			isChangeThePosition = False
			while(np.min(abs(np.append(NoEstimation.Node[i].pos[0]*np.ones((NoEstimation.Node[i].Ni,1)), NoEstimation.Node[i].pos[1]*np.ones((NoEstimation.Node[i].Ni,1)), axis=1) - NoEstimation.Node[i].pvicini)+2) <= 0.05):
				
				isChangeThePosition = True
				pos = NoEstimation.Node[i].pos[0] + (rand.uniform(1,2)-0.5)*0.2
				NoEstimation.Node[i].pos[i] = np.min(np.matrix('1 1'),np.max(np.matrix('0 0'),pos))
			
			if( isChangeThePosition ):
				print 'Overlapping-->'
				for j in range(NoEstimation.Node(i).vicini):
					NoEstimation.Node[j].pvicini[NoEstimation.Node(j).vicini == i,:] = pos
				print 'Solved'
		NoEstimation.CostFunction.append(computeCost(LocalParameter,NoEstimation.Node,TrueFunction,LocalParameter.CostFlag))
	return NoEstimation								

def computeEnergyFunction(V,Parameter,m=0,h=0,C=0):
	E = 0
	for i in range(Parameter.NumberOfAgents):
		E += 0.5*np.dot(V[i],np.transpose(V[i]))+9.8*m*h+C
	return E

def computeInitialCentralizedEstimation(RecursiveEstimation,Parameter):

	NumberOfLocations = RecursiveEstimation.InputLocations.shape[0]
	
	
	RecursiveEstimation.SampledKernel = computeKernel(RecursiveEstimation.InputLocations[:,0].reshape(NumberOfLocations,1),RecursiveEstimation.InputLocations[:,1].reshape(NumberOfLocations,1), Parameter.KernelStd)
	Autocovariance = np.linalg.inv(RecursiveEstimation.SampledKernel + Parameter.RegularizationConstant*np.eye(NumberOfLocations))
	RecursiveEstimation.InvertedSampledKernel = Autocovariance
				
	inputNoise = RecursiveEstimation.InputLocations[:,2].reshape(NumberOfLocations,1)

	EstimationCoefficients = np.dot(Autocovariance,inputNoise)
	RecursiveEstimation.DensityFunction = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
	for i in range(Parameter.NumberOfPoints):
		for j in range(Parameter.NumberOfPoints):
			power1 = np.power((Parameter.Xaxis[i]*np.ones((1,NumberOfLocations)) - \
							np.transpose(RecursiveEstimation.InputLocations[0:NumberOfLocations,0])),2)
			power1 = power1[0]
			power2 = np.power((Parameter.Yaxis[j]*np.ones((1,NumberOfLocations)) - \
							np.transpose(RecursiveEstimation.InputLocations[0:NumberOfLocations,1])),2)
			power2 = power2[0]
			KernelRow = np.exp(-(power1+power2)/(Parameter.KernelStd**2))
			KernelRow = KernelRow.reshape(1,NumberOfLocations)
			RecursiveEstimation.DensityFunction[j,i] = RecursiveEstimation.DensityFunction[j,i] + np.dot(KernelRow,EstimationCoefficients)
			RecursiveEstimation.APosterioriVariance[j,i] = 1 - np.dot(np.dot(KernelRow,Autocovariance),np.transpose(KernelRow))
	RecursiveEstimation.APosterioriVariancePure = RecursiveEstimation.APosterioriVariance[:]
	RecursiveEstimation.APosterioriVariance=np.multiply(Parameter.ObstacleRegionWithClearence,RecursiveEstimation.APosterioriVariance)

	return RecursiveEstimation

def computeKernel(X,Y,StdScale):
	N = len(X)
	K = np.exp( (-(np.power((X*np.ones((1,N)) - np.ones((N,1))*np.transpose(X)),2) + np.power((Y*np.ones((1,N)) - np.ones((N,1))*np.transpose(Y)),2)))/(StdScale**2))
	return K

def indices(a, func):
    return [i for (i, val) in enumerate(a) if func(val)]
	
def computeMaxLocationModified(EstimationStructure,Node,Parameter,Variance):
	MaxLocations = np.zeros((Parameter.NumberOfAgents,2),dtype='int')
	MaxLocationsValue = np.zeros((Parameter.NumberOfAgents,2),dtype='float')
	listsubmat = []
	mat = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
	r = Parameter.ws_model['robot_radius']/Parameter.RealScale
	c = Parameter.ws_model['robot_clearence']/Parameter.RealScale
	mindis = 2*(r+c)
	Node = EstimationStructure.Node
	numofagentsexploring = 0
	Centroid = np.zeros((Parameter.NumberOfAgents,2))
	for j in range(Parameter.NumberOfAgents):
		Centroid[j,:] = Node[j].C
	avEnergy = 0						
	Centroid_Obs = np.zeros((Parameter.NumberOfAgents,2))
	for j in range(Parameter.NumberOfAgents):
		Centroid_Obs[j,:] = Node[j].C_Obs
	xt = []
	yt = []
	VarianceTemp = copy.deepcopy(Variance)
	for i in range(Parameter.NumberOfAgents):
		submat = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
		subFunction = np.multiply(VarianceTemp,Node[i].VoronoiRegion)
		m = np.max(np.max(subFunction))
		tempsubFunction = subFunction.copy()
		subFunction = np.transpose(subFunction).reshape(subFunction.shape[0]*subFunction.shape[0],1)
		subsubFunction = []
		for j in range(len(subFunction)):
			if(subFunction[j] > 0.0):
				subsubFunction.append((subFunction[j],int(j)))
		ind = indices(subFunction, lambda x: x == m) 
		indx = []
		indy = []
		listind = []
		for j in range(len(ind)):
			indx.append(ind[j] // Parameter.NumberOfPoints)
			indy.append(ind[j] % Parameter.NumberOfPoints)
			listind.append(distance((float(indx[j])/Parameter.NumberOfPoints,float(indx[j])/Parameter.NumberOfPoints),Node[i].pos))
		
		
		dtype = [('val', float), ('idx', int)]
		values = np.array(subsubFunction,dtype=dtype)
		sortvalues = np.sort(values,order='val')
		for j in range((9*len(sortvalues))//10,len(sortvalues)): 	
			indsx=sortvalues[j][1] // Parameter.NumberOfPoints
			indsy=sortvalues[j][1] % Parameter.NumberOfPoints
			mat[indsx,indsy] = 1
			submat[indsx,indsy] = 1
		listsubmat.append(submat)
		minidx = np.argmax(listind) 
		MaxLocations[i,0] = indx[minidx] 
		MaxLocations[i,1] = indy[minidx]
		
		M = Parameter.UnitArea*sum(sum(submat))
		L0 = Parameter.UnitArea*sum(sum(np.multiply(Parameter.Xgrid,submat)))
		L1 = Parameter.UnitArea*sum(sum(np.multiply(Parameter.Ygrid,submat)))
		MaxLocationsValue[i,0] = L0/M 
		MaxLocationsValue[i,1] = L1/M
		if(EstimationStructure.statusFinishedAgent[i] == 0):
			MaxVariance = VarianceTemp[MaxLocations[i,1],MaxLocations[i,0]]
			if Parameter.Estimator == 'Gaussian':
				State = np.random.binomial(1,MaxVariance)
			else:
				State = rand.uniform(0,1) > 0.5
		elif(EstimationStructure.statusFinishedAgent[i] == 1):
			State = 1
		elif(EstimationStructure.statusFinishedAgent[i] == 2):
			State = 0
		if((Parameter.stopWithin == 'Iteration') and (Parameter.CurrentIteration > Parameter.Niter-2)):
			State = 0
		if(State == 1):
			EstimationStructure.statusFinishedAgent[i] = 1
			numofagentsexploring = numofagentsexploring +1
			# max location 
			pos = np.append(Parameter.Xgrid[0,MaxLocations[i,0]],\
				Parameter.Ygrid[MaxLocations[i,1],0])
			
			for p in range(Parameter.NumberOfPoints):
				for q in range(Parameter.NumberOfPoints):
					if(distance((Parameter.Xgrid[0,MaxLocations[i,0]],Parameter.Ygrid[MaxLocations[i,1],0]),(Parameter.Xgrid[0,p],Parameter.Ygrid[q,0])) < mindis):
						VarianceTemp[q,p] = -1
			
		else:
			EstimationStructure.statusFinishedAgent[i] = 2
			if(Node[i].C_Obs[0] != None):
				pos = Centroid_Obs[i,:]		
			else:
				pos = Centroid[i,:]
		avEnergy = avEnergy + np.linalg.norm(pos-Node[i].pos)/Parameter.NumberOfAgents
		Node[i].pos = pos

	EstimationStructure.averageEnergy.append(avEnergy)	
	EstimationStructure.NumberOfAgentsExploring.append(numofagentsexploring)	
	newCurrentPositions = node2pos(Node)
	for i in range(Parameter.NumberOfAgents):
		Node[i].pvicini = newCurrentPositions[Node[i].vicini-1,:]
	for i in range(Parameter.NumberOfAgents):
		# flag isChangedThePosition OFF
		isChangeThePosition = False
		while(np.min(abs(np.append(Node[i].pos[0]*np.ones((Node[i].Ni,1)), Node[i].pos[1]*np.ones((Node[i].Ni,1)), axis=1) - Node[i].pvicini)+2) <= 0.05):
			# flag isChangedThePosition ON
			isChangeThePosition = True
			# update to a random position close to the current position
			pos = NoEstimation.Node[i].pos[0] + (rand.uniform(1,2)-0.5)*0.2
			# checking the boundary and fitting the position into the grid
			NoEstimation.Node[i].pos[i] = np.min(np.matrix('1 1'),np.max(np.matrix('0 0'),pos))
		
		if( isChangeThePosition ):
			print 'Overlapping-->'
			for j in range(NoEstimation.Node(i).vicini):
				NoEstimation.Node[j].pvicini[NoEstimation.Node(j).vicini == i,:] = pos
			print 'Solved'
	'''
	fig2=plt.figure()
	ax2 = plt.subplot()
	extent = 0,1,0,1
	plotvororeg = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
	for i in range(Parameter.NumberOfAgents):
		plotvororeg = EstimationStructure.Node[i].VoronoiRegionWithoutObstacle.astype(int)*((i)*20)+plotvororeg
	ax2.imshow(np.flipud(plotvororeg),extent=extent,cmap='Pastel1')
	CS = ax2.contour(Parameter.Xgrid, Parameter.Ygrid, Variance)
	cbar = plt.colorbar(CS)
	cbar.ax.set_ylabel('verbosity coefficient')
	# Add the contour line levels to the colorbar
	cbar.add_lines(CS)
	xpos = []
	ypos = []
	for i in range(Parameter.NumberOfAgents):
		xpos.append(Node[i].pos[0])
		ypos.append(Node[i].pos[1])
	ax2.scatter(xpos, ypos)
	for i in range(Parameter.NumberOfAgents):
		ax2.annotate(str(i), (xpos[i],ypos[i]))
	
	for hole in Parameter.ws_model['circular_obstacles']:
		srec = matplotlib.patches.Circle(
				(hole[0], hole[1]),
				1.2*hole[2],
				facecolor= 'black',
				fill = True,
				alpha=1)
		ax2.add_patch(srec)

	ax2.set_xlim((0,1))
	ax2.set_ylim((0,1))
	ax2.clabel(CS, inline=1, fontsize=10)
	plt.title('Updated Variance of Density Function')
	namebuf = "varplot-%d.png" % Parameter.CurrentIteration

	plt.savefig(namebuf)
	plt.close()
	fig2.clf()
	'''
	EstimationStructure.Node = Node
	return EstimationStructure


def computeMaxLocation(Node,Parameter,Variance):
	
	MaxLocations = np.zeros((Parameter.NumberOfAgents,2),dtype='int')
	MaxLocationsValue = np.zeros((Parameter.NumberOfAgents,2),dtype='float')
	#Variance = np.transpose(Variance)
	# extract the gradient for each node
	listsubmat = []
	mat = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
	for i in range(Parameter.NumberOfAgents):
		submat = np.zeros((Parameter.NumberOfPoints,Parameter.NumberOfPoints))
		subFunction = np.multiply(Variance,Node[i].VoronoiRegion)
		#subFunction[0,0] = 1.0;
		#subFunction[0,1] = 1.0;
		#[~,ind] = max(subFunction(:));
		'''
		fig2=plt.figure()
		ax2 = plt.subplot()
		#voronoi_plot_2d(vor,ax=ax2)
		extent = 0,1,0,1
		#ax2.imshow(np.flipud(plotvororeg),extent=extent,cmap=colormap)
		#CS = ax2.contour(Parameter.Xgrid, Parameter.Ygrid, RecursiveEstimation.APosterioriVariance)
	
		ax2.matshow(Node[i].VoronoiRegion,zorder=1)

		#ax2.clabel(CS, inline=1, fontsize=10)
		plt.title('Estimation of Variance of Density Function (Pixelize)')

		plt.show()
		plt.close()
		fig2.clf()
		'''
		m = np.max(np.max(subFunction))
		#print Variance
		#print subFunction
		tempsubFunction = subFunction.copy()
		subFunction = np.transpose(subFunction).reshape(subFunction.shape[0]*subFunction.shape[0],1)
		subsubFunction = []
		for j in range(len(subFunction)):
			if(subFunction[j] > 0.0):
				subsubFunction.append((subFunction[j],int(j)))
		#print 'm',m
		#print subFunction
		ind = indices(subFunction, lambda x: x == m) #subFunction.tolist().index(m)#find(subFunction(:) == m)
		indx = []
		indy = []
		listind = []
		for j in range(len(ind)):
			indx.append(ind[j] // Parameter.NumberOfPoints)
			indy.append(ind[j] % Parameter.NumberOfPoints)
			listind.append(distance((float(indx[j])/Parameter.NumberOfPoints,float(indx[j])/Parameter.NumberOfPoints),Node[i].pos))
		
		
		dtype = [('val', float), ('idx', int)]
		#tvalues = []
		#for j in range(len(subFunction)):
		#	tvalues.append((subFunction[j],j))
		values = np.array(subsubFunction,dtype=dtype)
		#print 'ssf',sortSubFunction
		sortvalues = np.sort(values,order='val')
		for j in range((9*len(sortvalues))//10,len(sortvalues)): 	
			indsx=sortvalues[j][1] // Parameter.NumberOfPoints
			indsy=sortvalues[j][1] % Parameter.NumberOfPoints
			mat[indsx,indsy] = 1
			submat[indsx,indsy] = 1
		listsubmat.append(submat)
		minidx = np.argmax(listind) # np.argmin(listind) # np.argsort(listind)[len(listind)//2]#
		MaxLocations[i,0] = indx[minidx] 
		MaxLocations[i,1] = indy[minidx]
		#print 'maxl',MaxLocations[i]
		
		M = Parameter.UnitArea*sum(sum(submat))
		L0 = Parameter.UnitArea*sum(sum(np.multiply(Parameter.Xgrid,submat)))
		L1 = Parameter.UnitArea*sum(sum(np.multiply(Parameter.Ygrid,submat)))
		MaxLocationsValue[i,0] = L0/M 
		MaxLocationsValue[i,1] = L1/M
		
		#print 'maxlv',L0/M , L1/M 
		
		'''
		print ind, indx, indy
		n = len(ind)
		#print 'n'
		#print n
		ind = ind[rand.randint(0,n-1)]
		print ind
		nodeMaxX,nodeMaxY = np.unravel_index(ind,Variance.shape)
		#print nodeMaxX
		#print nodeMaxY
		#if Parameter.CurrentIteration == 2
		#sys.exit()
		MaxLocations[i,0] = nodeMaxX
		MaxLocations[i,1] = nodeMaxY
		'''
	#sys.exit()
	return MaxLocations,MaxLocationsValue,mat,listsubmat
	
def computeSensoryFunction(x,y,LocalParameter):
	GridEnd = LocalParameter.GridEnd
	#nx = 1 #len(x)
	#ny = 1 #len(y)
	#f = np.zeros((nx,ny))
	#for i in range(0,nx):
	#	for j in range(0,ny):
	#f = 20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+5*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+5*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)
	#fun1:
	#f = 20*np.exp(-((x-GridEnd*0.5)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.85)**2+(y-GridEnd*0.3)**2)/(GridEnd*0.2)**2)+12*np.exp(-((x-GridEnd*0.5)**2+(y-GridEnd*0.6)**2)/(GridEnd*0.2)**2)
	#fun2:
	#f = 20*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+20*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)+7*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)+7*np.exp(-((x-GridEnd*0.2)**2+(y-GridEnd*0.8)**2)/(GridEnd*0.2)**2)
	# fun3: 2 titik
	f = 20*np.exp(-((x-GridEnd*0.35)**2+(y-GridEnd*0.65)**2)/(GridEnd*0.2)**2)+12*np.exp(-((x-GridEnd*0.8)**2+(y-GridEnd*0.2)**2)/(GridEnd*0.2)**2)
	return f

def computeUpdate(EstimationStructure,Parameter,MaxLocations,MaxLocationsValue,Variance):
	
	Node = EstimationStructure.Node
	numofagentsexploring = 0
	# computes the centroids for all the agents
	Centroid = np.zeros((Parameter.NumberOfAgents,2))
	for j in range(Parameter.NumberOfAgents):
		Centroid[j,:] = Node[j].C
	avEnergy = 0						
	Centroid_Obs = np.zeros((Parameter.NumberOfAgents,2))
	for j in range(Parameter.NumberOfAgents):
		Centroid_Obs[j,:] = Node[j].C_Obs
	for i in range(Parameter.NumberOfAgents):
		#print Variance
		#print MaxLocations
		if(EstimationStructure.statusFinishedAgent[i] == 0):
			MaxVariance = Variance[MaxLocations[i,1],MaxLocations[i,0]]
			#print 'maxv',MaxVariance
			if Parameter.Estimator == 'Gaussian':
				State = np.random.binomial(1,MaxVariance)
				#State = rand.uniform(0,1) > np.power(MaxVariance,(Parameter.VarianceExponentialShaping))
				#State = Parameter.TradeOffCoeff > MaxVariance
				#if(MaxVariance > Parameter.TradeOffCoeff):
				#	State = 1
				#else:
				#	State = np.random.binomial(1,MaxVariance)
			else:
				State = rand.uniform(0,1) > 0.5
		elif(EstimationStructure.statusFinishedAgent[i] == 1):
			State = 1
		elif(EstimationStructure.statusFinishedAgent[i] == 2):
			State = 0
		#State = 0.5 > np.power(MaxVariance,(Parameter.VarianceExponentialShaping))
		# update of the positon of the node i
		if((Parameter.stopWithin == 'Iteration') and (Parameter.CurrentIteration > Parameter.Niter-2)):
			State = 0
		#if((Parameter.stopWithin == 'MaxVariance') and (EstimationStructure.maxAPosterioriVariance[Parameter.CurrentIteration-2] < Parameter.stopMaxVariance)):
		#	State = 0
		if(State == 1): # and (Parameter.stopWithin == 'Iteration') and (Parameter.CurrentIteration < Parameter.Niter-2)):
			EstimationStructure.statusFinishedAgent[i] = 1
			# updating the number of agents exploring
			numofagentsexploring = numofagentsexploring +1
			#EstimationStructure.NumberOfAgentsExploring[0,Parameter.CurrentIteration] = \
			#	EstimationStructure.NumberOfAgentsExploring[0,Parameter.CurrentIteration] + 1
					
			# max location 
			pos = np.append(Parameter.Xgrid[0,MaxLocations[i,0]],\
				Parameter.Ygrid[MaxLocations[i,1],0])
			#pos = np.append(MaxLocationsValue[i,0],MaxLocationsValue[i,1])
			#print 'pos',pos
			
			# saturator
			#pos = min([Parameter.GridEnd Parameter.GridEnd],\
			#		max([Parameter.GridStart Parameter.GridStart],pos));
		else:
			EstimationStructure.statusFinishedAgent[i] = 2
			if(Node[i].C_Obs[0] != None):
				pos = Centroid_Obs[i,:]		
			else:
				pos = Centroid[i,:]
			#print Centroid[i,:]
			#print Node[i].C_Obs
	
		# saturate motion if limited capabilities
	#     if norm(Node(i).pos - pos) > Parameter.MotionConstraintsThreshold
	#         
	#         slope = atan2((pos(2) - Node(i).pos(2)),(pos(1) - Node(i).pos(1)));
	#         pos_sat = Node(i).pos + Parameter.MotionConstraintsThreshold * ...
	#                                     [real(exp(1i*slope)) imag(exp(1i*slope))];
	#         pos = pos_sat;
	#     end
		avEnergy = avEnergy + np.linalg.norm(pos-Node[i].pos)/Parameter.NumberOfAgents
		#EstimationStructure.averageEnergy.append(EstimationStructure.averageEnergy[Parameter.CurrentIteration] + np.linalg.norm(pos-Node[i].pos)/Parameter.NumberOfAgents)
		
		Node[i].pos = pos
		#print pos
	EstimationStructure.averageEnergy.append(avEnergy)	
	EstimationStructure.NumberOfAgentsExploring.append(numofagentsexploring)	
	#sys.exit()
	# update neighbors positions
	newCurrentPositions = node2pos(Node)
	for i in range(Parameter.NumberOfAgents):
	
		Node[i].pvicini = newCurrentPositions[Node[i].vicini-1,:]
	
	
	# Checking if two or more nodes are in the same position
	for i in range(Parameter.NumberOfAgents):
		
		# flag isChangedThePosition OFF
		isChangeThePosition = False
		#print np.min(abs(np.append(NoEstimation.Node[i].pos[0]*np.ones((NoEstimation.Node[i].Ni,1)), NoEstimation.Node[i].pos[1]*np.ones((NoEstimation.Node[i].Ni,1)), axis=1)- NoEstimation.Node[i].pvicini)+2)
		while(np.min(abs(np.append(Node[i].pos[0]*np.ones((Node[i].Ni,1)), Node[i].pos[1]*np.ones((Node[i].Ni,1)), axis=1) - Node[i].pvicini)+2) <= 0.05):
			
			# flag isChangedThePosition ON
			isChangeThePosition = True
			
			# update to a random position close to the current position
			pos = NoEstimation.Node[i].pos[0] + (rand.uniform(1,2)-0.5)*0.2
			#print pos
			# checking the boundary and fitting the position into the grid
			NoEstimation.Node[i].pos[i] = np.min(np.matrix('1 1'),np.max(np.matrix('0 0'),pos))
		
		if( isChangeThePosition ):
			print 'Overlapping-->'
			for j in range(NoEstimation.Node(i).vicini):
				NoEstimation.Node[j].pvicini[NoEstimation.Node(j).vicini == i,:] = pos
			print 'Solved'
	
	EstimationStructure.Node = Node
	return EstimationStructure
	
def computeVoronoiRegion(Node, LocalParameter,ws_model):
	#Node = LocalNode
	#print Node[0]
	#print LocalParameter.VoronoiMode
	if (LocalParameter.VoronoiMode == 'Positions'):
		#print 'Positions'
		for i in range(LocalParameter.NumberOfAgents):
			#print Node[i].Ni
			#print Node[i].VoronoiRegion.shape
			#print np.ones((LocalParameter.NumberOfPoints, LocalParameter.NumberOfPoints)).shape
			#Node[i].VoronoiRegion 
			Node[i].VoronoiRegion = np.ones((LocalParameter.NumberOfPoints, LocalParameter.NumberOfPoints))
			
			for j in range(Node[i].Ni):
				#print Node[i].pos[1]
				#print Node[i].pvicini[j,1]
				if(Node[i].pos[1] == Node[i].pvicini[j,1]):
					Intercept = (Node[i].pos[0] + Node[i].pvicini[j,0])/2
					if (Node[i].pos[0] - Intercept >= 0):
						tempVoronoi = (LocalParameter.Xgrid - Intercept) >= np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
					else:
						tempVoronoi = (LocalParameter.Xgrid - Intercept) < np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
				else:
					Slope = (Node[i].pos[0]-Node[i].pvicini[j,0])/(Node[i].pvicini[j,1]-Node[i].pos[1])
					Intercept = (Node[i].pos[1]+Node[i].pvicini[j,1])/2 - Slope*(Node[i].pos[0]+Node[i].pvicini[j,0])/2
					if ((Node[i].pos[1] - Slope*Node[i].pos[0] - Intercept) >= 0):
						tempVoronoi = (LocalParameter.Ygrid - Slope*LocalParameter.Xgrid - Intercept) >= np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
					else:
						tempVoronoi = (LocalParameter.Ygrid - Slope*LocalParameter.Xgrid - Intercept) < np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
				
				Node[i].VoronoiRegion = np.bitwise_and(Node[i].VoronoiRegion == 1,tempVoronoi)
			Node[i].VoronoiRegionWithoutObstacle = Node[i].VoronoiRegion
			#Node[i].VoronoiRegion = Node[i].VoronoiRegion.astype(float)
			#print Node[i].VoronoiRegion.astype(int)
			'''
			fig = plt.figure()
			ax = fig.gca(projection='3d')
			
			# Plot the surface.
			surf = ax.plot_surface(LocalParameter.Xgrid, LocalParameter.Ygrid, Node[i].VoronoiRegion.astype(int), cmap=cm.coolwarm,
								linewidth=0, antialiased=False)
			
			# Add a color bar which maps values to colors.
			fig.colorbar(surf, shrink=0.5, aspect=5)
			
			plt.show()
			'''
	else:
		Centroids = np.zeros((LocalParameter.NumberOfAgents,2))
		for i in range(LocalParameter.NumberOfAgents):
			Centroids[i,:] = Node[i].C
		#print 'Centroids'
		for i in range(LocalParameter.NumberOfAgents):
			Node[i].VoronoiRegion = np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
			for j in range(Node[i].Ni): # loop neighbor j
				if (Centroids[i,1]==Centroids[Node[i].vicini[j]-1,1]): # jika centroid agent[y] sama dengan centroid neighbor[y] 
					Intercept = (Centroids[i,0] + Centroids[Node[i].vicini[j]-1,0])/2
					if (Centroids[i,0] - Intercept >=0):
						tempVoronoi = (LocalParameter.Xgrid - Intercept) >= np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
					else:
						tempVoronoi = (LocalParameter.Xgrid - Intercept) < np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
				else:
					Slope = (Centroids[i,0]-Centroids[Node[i].vicini[j]-1,0])/(Centroids[Node[i].vicini[j]-1,1]-Centroids[i,1])
					Intercept = (Centroids[i,1]+Centroids[Node[i].vicini[j]-1,1])/2 - Slope*(Centroids[i,0]+Centroids[Node[i].vicini[j]-1,0])/2
					if ((Centroids[i,1] - Slope*Centroids[i,0] - Intercept) >= 0):
						tempVoronoi = (LocalParameter.Ygrid - Slope*LocalParameter.Xgrid - Intercept) >= np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
					else:
						tempVoronoi = (LocalParameter.Ygrid - Slope*LocalParameter.Xgrid - Intercept) < np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
				
				Node[i].VoronoiRegion = np.bitwise_and(Node[i].VoronoiRegion == 1,tempVoronoi)
			Node[i].VoronoiRegionWithoutObstacle = Node[i].VoronoiRegion
			#Node[i].VoronoiRegion = Node[i].VoronoiRegion.astype(float)
			#print sum(sum(Node[i].VoronoiRegion))
			'''
			fig = plt.figure()
			ax = fig.gca(projection='3d')
			
			# Plot the surface.
			surf = ax.plot_surface(LocalParameter.Xgrid, LocalParameter.Ygrid, Node[i].VoronoiRegion.astype(int), cmap=cm.coolwarm,
								linewidth=0, antialiased=False)
			
			# Add a color bar which maps values to colors.
			fig.colorbar(surf, shrink=0.5, aspect=5)
			
			plt.show()
			'''
	
	ObstacleRegion = LocalParameter.ObstacleRegionWithClearence;
		#np.savetxt('voronoi/obs.out',ObstacleRegion,fmt='%i')
	for hole in ws_model['rectangle_obstacles']:
		if ws_model['type_obstacle'][hole_idx] == 's' :
			for i in range(LocalParameter.NumberOfPoints):
				for j in range(LocalParameter.NumberOfPoints):
					#dist = np.sqrt((ax[i]-hole[0])*(ax[i]-hole[0])+(ay[j]-hole[1])*(ay[j]-hole[1]))
					#if (dist < (1.2*hole[2]+0.01)):
					#	ObstacleRegion[j,i]=0
					if ((ax[i] > hole[0]) and (ax[i] < hole[0]+hole[2]) and (ay[j] > hole[1]) and (ay[j] < hole[1]+hole[3])):
						ObstacleRegion[j,i] = 0


	#print Node[0].VoronoiRegion.shape
	for i in range(LocalParameter.NumberOfAgents):
		Node[i].VoronoiRegion = np.bitwise_and(Node[i].VoronoiRegion == 1,ObstacleRegion == 1)
		Node[i].VoronoiRegion = Node[i].VoronoiRegion.astype(float)
		#print Node[i].VoronoiRegion.astype(int)

	#print 'test1',Node[0].VoronoiRegion
	if(LocalParameter.ModifiedVoronoi and (LocalParameter.CurrentIteration > 1)):
		listV = []
		for i in range(LocalParameter.NumberOfAgents):
			vortemp = Node[i].VoronoiRegion.copy()
			V,c = countregion(vortemp)
			listV.append(V)
			Node[i].creg = c
		#print 'test2',Node[0].VoronoiRegion
		for i in range(LocalParameter.NumberOfAgents):
			if(Node[i].creg > 1):
				listv = []
				for j in range(Node[i].creg):
					v = 1*(listV[i] == (np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))+j+2))
					listv.append(v)
				listmindisc = []
				for w in listv:
					mindisc = np.inf
					for p in range(LocalParameter.NumberOfPoints):
						for q in range(LocalParameter.NumberOfPoints):
							if(w[p,q] == 1):
								disc = distancetuple((p,q),Node[i].pos)
								if(mindisc > disc):
									mindisc = disc
					listmindisc.append(mindisc)
				#print 'test1',Node[i].VoronoiRegion
				Node[i].VoronoiRegion = listv[np.argmin(listmindisc)]
				#print 'test2',Node[i].VoronoiRegion
				del listv[np.argmin(listmindisc)]
			
				for w in listv:
					c = 0
					for j in range(LocalParameter.NumberOfAgents):
						if (i!=j):
							#print w
							#print Node[j].VoronoiRegion
							newv = np.add(w,Node[j].VoronoiRegion)
							#print newv
							V,c = countregion(newv)
						if(c == 1):
							#if(i == 3):
							#	print '================'
							#	print j,c
							#	print V
							#	print newv
							#	print w
							#	print Node[j].VoronoiRegion
								#sys.exit()
							Node[j].VoronoiRegion=newv
							break



	'''
	for i in range(LocalParameter.NumberOfAgents):
		np.savetxt('voronoi/voronoi'+repr(i)+'.out',Node[i].VoronoiRegion.astype(int),fmt='%i')
		np.savetxt('voronoi/voronoiwo'+repr(i)+'.out',Node[i].VoronoiRegionWithoutObstacle.astype(int),fmt='%i')
	'''
	#sys.exit()
	return Node


def computeCentroid(Node,LocalParameter,DensityFunction,ws_model):
	for i in range(DensityFunction.shape[0]):
		for j in range(DensityFunction.shape[1]):
			if DensityFunction[i,j] < 0:
				DensityFunction[i,j] = 1e-10
	#print DensityFunction 
	#print Node[0].VoronoiRegion
	for i in range(LocalParameter.NumberOfAgents):
		#print Node[i].VoronoiRegion
		Node[i].M = LocalParameter.UnitArea*sum(sum(np.multiply(DensityFunction,Node[i].VoronoiRegion)))
		Node[i].L[0] = LocalParameter.UnitArea*sum(sum(np.multiply(np.multiply(LocalParameter.Xgrid,DensityFunction),Node[i].VoronoiRegion)))
		Node[i].L[1] = LocalParameter.UnitArea*sum(sum(np.multiply(np.multiply(LocalParameter.Ygrid,DensityFunction),Node[i].VoronoiRegion)))
		Node[i].C = Node[i].L / Node[i].M

		#print 'Centroid Print'
		#print Node[i].L
		#print Node[i].C
	hole_idx = 0
	xgrid2 = np.multiply(LocalParameter.Xgrid,LocalParameter.Xgrid)
	ygrid2 = np.multiply(LocalParameter.Ygrid,LocalParameter.Ygrid)
	for i in range(LocalParameter.NumberOfAgents):
		Node[i].C_Obs = [None,None]
		for hole in ws_model['circular_obstacles']:
			dist = np.sqrt((Node[i].C[0]-hole[0])*(Node[i].C[0]-hole[0])+(Node[i].C[1]-hole[1])*(Node[i].C[1]-hole[1]))
			if (dist < (1.2*hole[2]+0.01)): # real centroid inside obstacle
				xC = np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))+Node[i].C[0]
				yC = np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))+Node[i].C[1]
				distx2 = np.multiply(LocalParameter.Xgrid-xC,LocalParameter.Xgrid-xC)
				disty2 = np.multiply(LocalParameter.Ygrid-yC,LocalParameter.Ygrid-yC)
				distxy = np.sqrt(distx2+disty2)
				#weighting
				if(LocalParameter.CentroidNonConvex == 'Weighting'):		
					qwerty = 0.1*np.multiply(DensityFunction,Node[i].VoronoiRegion)-5*distxy
					maxqwertyidx = np.argmax(qwerty)
					#print maxqwertyidx
					Node[i].C_Obs[0] = LocalParameter.Yaxis[maxqwertyidx % LocalParameter.NumberOfPoints]
					Node[i].C_Obs[1] = LocalParameter.Xaxis[maxqwertyidx // LocalParameter.NumberOfPoints]
				elif(LocalParameter.CentroidNonConvex == 'MinCost'):
					densitymat = np.multiply(DensityFunction,Node[i].VoronoiRegion)
					sumcost = np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
					for p in range(LocalParameter.NumberOfPoints):
						for q in range(LocalParameter.NumberOfPoints):
							if(densitymat[p,q] > 0):
								for r in range(LocalParameter.NumberOfPoints):
									for s in range(LocalParameter.NumberOfPoints):
										if(densitymat[r,s] > 0):
											cost = ((p-r)*(p-r)+(q-s)*(q-s))*densitymat[r,s] #(M[i,j] - M[p,q])*(M[i,j] - M[p,q])
											sumcost[p,q] = sumcost[p,q] + cost
					for p in range(LocalParameter.NumberOfPoints):
						for q in range(LocalParameter.NumberOfPoints):
							if(sumcost[p,q] == 0):
								sumcost[p,q] = 99999999;
					Node[i].C_Obs[0] = LocalParameter.Yaxis[np.argmin(sumcost) % LocalParameter.NumberOfPoints]
					Node[i].C_Obs[1] = LocalParameter.Xaxis[np.argmin(sumcost) // LocalParameter.NumberOfPoints]
					#print "C OBS"
					#print Node[i].C_Obs[0],Node[i].C_Obs[1]
				else:
					#median
					maxdist = np.zeros((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))+np.max(distxy)
					densitymatV = np.multiply(np.multiply(DensityFunction,Node[i].VoronoiRegion),maxdist-distxy)
					#print densitymatV
					cartList = []
					dataList = []
					cartList.append([0,0])
					dataList.append(0)
					for p in range(LocalParameter.NumberOfPoints):
						for q in range(LocalParameter.NumberOfPoints):
							if(densitymatV[p][q] != 0):
								for k in range(len(cartList)):
									if(dataList[k] < densitymatV[p][q]):
										cartList.insert(k,[p,q])
										dataList.insert(k,densitymatV[p,q])
										break;
					del dataList[-1]
					del cartList[-1]			
					#print dataList
					#print len(dataList)
					#print dataList[len(dataList)//2]
					#print cartList[len(dataList)//2]
					#sys.exit()
					Node[i].C_Obs[0] = LocalParameter.Yaxis[cartList[len(dataList)//2][1]]
					Node[i].C_Obs[1] = LocalParameter.Xaxis[cartList[len(dataList)//2][0]]
				#print LocalParameter.Yaxis[minqwertyidx % LocalParameter.NumberOfPoints]
				#print LocalParameter.Xaxis[minqwertyidx // LocalParameter.NumberOfPoints]
				#print 'centroid in obs-'+repr(hole_idx)+' agent-'+repr(i)
			hole_idx += 1 

	#print Node
	#print 'test'
	return Node

def generalSchurInverse(Ainv,B,D):
	lenA = len(Ainv)
	
	lenD = len(D)
	#print 'enD'
	#print lenD
	Minv = np.zeros((lenA+lenD,lenA+lenD))
	
	AinvB = np.dot(Ainv,B)
	#print 'AinvB'
	#print AinvB
	M22notInv = D - np.dot(np.transpose(B),AinvB)
	M22Inv=np.linalg.inv(M22notInv)
	#M22B,resid,rank,s = np.linalg.lstsq(M22notInv,np.transpose(B))
	M22B = np.dot(M22Inv,np.transpose(B))
	#print 'M22B'
	#print M22B
	#M22B = M22notInv\B';
	end = Minv.shape[0]
	
	Minv[end-lenD:end,end-lenD:end] = M22Inv
	tmp = np.dot(AinvB,np.dot(M22B,Ainv))
	Minv[0:lenA,0:lenA] = Ainv + tmp
	#print sum(-np.dot(M22Inv,AinvB))
	Minv[0:lenA,end-lenD:end] = -np.dot(AinvB,M22Inv)
	Minv[end-lenD:end,0:lenA] = -np.dot(M22B,Ainv)
	#print sum(Minv)
	return Minv

def kernelEvaluation(Xaxis,Yaxis,InputLocations,KernelStd,lastKbar,numofagents):
	N=len(Xaxis)
	M=InputLocations.shape[0]
	#ct.memset
	kernelout = np.zeros((M,N*N))
	#arr = (ct.c_double * len(kernelout))(kernelout)
	
	kernelrowspent_sum=0
	#print 'Kbar value:'+repr(lastKbar)
	#print lastKbar.shape
	if (lastKbar.shape[0] == 1): # iterasi pertama
		for i in range(N):
			for j in range(N):
				st = time.time()
				
				#print "lastKbar "+repr(lastKbar.shape)
				kernelrow = np.zeros((M,1))
				kernelrow=vectorExponential0(Xaxis[i],Yaxis[j],InputLocations,kernelrow,M,KernelStd)
				
				kernelrowspent = time.time()-st
				kernelrowspent_sum = kernelrowspent_sum+kernelrowspent
				#print i*N*M+j*M
				#print kernelrow.shape
				#print kernelout[:,i*N+j:i*N+j+1].shape
				#print ct.addressof(kernelout.ctypes.data)
				#ct.memmove((kernelout.ctypes.data+i*N*M+j*M),kernelrow.ctypes.data,M*ct.sizeof(ct.c_double))
				kernelout[:,i*N+j:i*N+j+1] = kernelrow	
	else:
		for i in range(N):
			for j in range(N):
				st = time.time()
				#lastkernelrow = lastKbar[:,i+N*j]
				kernelrow = np.zeros((M,1))
				
				#if(i+N*j == 0):
				#	print (kernelrow) 
				#if((N-1)+N*(N-1) == i+N*j):
				#	print sum(kernelrow) 
				#print "lastKbar "+repr(lastKbar.shape)
				
				kernelrow=vectorExponential(Xaxis[i],Yaxis[j],InputLocations,kernelrow,M,KernelStd,numofagents)
				kernelrow[0:M-numofagents,0]=lastKbar[:,i*N+j]
				kernelrowspent = time.time()-st
				kernelrowspent_sum = kernelrowspent_sum+kernelrowspent
				#print i*N*M+j*M
				#print kernelrow.shape
				#print kernelout[:,i*N+j:i*N+j+1].shape
				#print ct.addressof(kernelout.ctypes.data)
				#ct.memmove((kernelout.ctypes.data+i*N*M+j*M),kernelrow.ctypes.data,M*ct.sizeof(ct.c_double))
				kernelout[:,i*N+j:i*N+j+1] = kernelrow
	#print "lastkernelrow shape "+repr(lastkernelrow.shape)
	#print "kernelrow shape "+repr(kernelrow.shape)
	#print 'kernelrow spent'+repr(kernelrowspent_sum)
	#print InputLocations
	return kernelout
	
def nodeInit(Positions,LocalParameter):
	MyStruct = recordtype("MyStruct", "vicini Ni pvicini pos VoronoiRegion VoronoiRegionWithoutObstacle M L C C_Obs creg")
	node = []
	for i in range(LocalParameter.NumberOfAgents):
		vicini = np.arange(1,LocalParameter.NumberOfAgents+1,1)
		vicini_idx = np.where(vicini == i+1)[0]
		vicini = np.delete(vicini,vicini_idx)
		#print vicini
		Ni = len(vicini)
		pvicini = np.delete(Positions,(vicini_idx),axis=0)
		#print pvicini
		temppos = Positions[vicini_idx,:]

		pos = [temppos[0,0],temppos[0,1]]
		
		#print pos
		VoronoiRegion = np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
		VoronoiRegionWithoutObstacle = np.ones((LocalParameter.NumberOfPoints,LocalParameter.NumberOfPoints))
		#print LocalParameter.NumberOfPoints
		node.append(MyStruct(vicini,Ni,pvicini,pos,VoronoiRegion,VoronoiRegionWithoutObstacle,None,[None,None],[None,None],[None,None],1))
	return node

def node2pos(node):
	N=len(node) # N=num of agent
	Position = np.zeros((N,2))
	
	for i in range(N):
		Position[i,:] = node[i].pos
		
	return Position
	
def transmitNewMeasurements(Parameter, EstimationStructure):
	# This function simulate data transmission from 
	# the nodes to the Central base station.

    # getting the positions of the nodes
	#print len(EstimationStructure.Node)
	PositionRecursiveEstimation = node2pos(EstimationStructure.Node)

    # new measurements
	noisyMeasurements = np.zeros((Parameter.NumberOfAgents,1))
	for i in range(Parameter.NumberOfAgents):
		
		noisyMeasurements[i] = computeSensoryFunction(PositionRecursiveEstimation[i,0],PositionRecursiveEstimation[i,1], Parameter)+Parameter.NoiseStd*np.random.normal(0,1)
		#print 'noisy'
		#print noisyMeasurements[i]
    # take new measurements in the new input locations

	#print Parameter.CurrentIteration
	if (Parameter.CurrentIteration == 1):
		EstimationStructure.InputLocations = np.append(PositionRecursiveEstimation,noisyMeasurements,axis=1)
	else:
		EstimationStructure.InputLocations = np.append(EstimationStructure.InputLocations,np.append(PositionRecursiveEstimation,noisyMeasurements,axis=1),axis=0)
	#print '----'
	#print EstimationStructure.InputLocations
	#print '----'
	return EstimationStructure

def transmitNewMeasurementOneAgent(Parameter,x):
	return computeSensoryFunction(x[0],x[1], Parameter)+Parameter.NoiseStd*np.random.normal(0,1)

def joinNewMeasurements(Parameter, EstimationStructure,sensor,pos_x,pos_y):
	for i in range(Parameter.NumberOfAgents):
		EstimationStructure.Node[i].pos[0] = pos_x[i]
		EstimationStructure.Node[i].pos[1] = pos_y[i]

	PositionRecursiveEstimation = node2pos(EstimationStructure.Node)

    # new measurements
	noisyMeasurements = np.zeros((Parameter.NumberOfAgents,1))
	for i in range(Parameter.NumberOfAgents):
		
		noisyMeasurements[i] = sensor[i]
		#print 'noisy'
		#print noisyMeasurements[i]
    # take new measurements in the new input locations

	#print Parameter.CurrentIteration
	if (Parameter.CurrentIteration == 1):
		EstimationStructure.InputLocations = np.append(PositionRecursiveEstimation,noisyMeasurements,axis=1)
	else:
		EstimationStructure.InputLocations = np.append(EstimationStructure.InputLocations,np.append(PositionRecursiveEstimation,noisyMeasurements,axis=1),axis=0)
	#print '----'
	#print EstimationStructure.InputLocations
	#print '----'
	return EstimationStructure

def vectorExponential(xAxis,yAxis,inputLocations,rowVector,n,scale,numofagents):
	#print inputLocations
	#print rowVector.shape
	scale_2 = scale**2

	for j in range(n-numofagents,n):
	#for j in range(n):
		#print j
		#print inputLocations[i,0]
		#print inputLocations[i,1]
		#print np.exp(-(np.power(xAxis-inputLocations[i,0],2) + np.power(yAxis-inputLocations[i,1],2))/(scale**2))
		#rowVector[i,0] = np.exp(-(np.power(xAxis-inputLocations[i,0],2) + np.power(yAxis-inputLocations[i,1],2))/(scale_2))
		rowVector[j,0] = np.exp(-(np.multiply(xAxis-inputLocations[j,0],xAxis-inputLocations[j,0]) + np.multiply(yAxis-inputLocations[j,1],yAxis-inputLocations[j,1]))/(scale_2))

	return rowVector

def vectorExponential0(xAxis,yAxis,inputLocations,rowVector,n,scale):
	#print inputLocations
	#print rowVector.shape
	scale_2 = scale**2
	#print n
	#print "lastkernelrow "+repr(lastkernelrow.shape)
	#print "rowVector "+repr(rowVector.shape)
	#rowVector = lastkernelrow
	
	#rV = np.zeros((lP.Parameter.NumberOfAgents,lP.Parameter.NumberOfPoints*lP.Parameter.NumberOfPoints)) 
	for i in range(n):
		#print inputLocations[i,0]
		#print inputLocations[i,1]
		#print np.exp(-(np.power(xAxis-inputLocations[i,0],2) + np.power(yAxis-inputLocations[i,1],2))/(scale**2))
		#rowVector[i,0] = np.exp(-(np.power(xAxis-inputLocations[i,0],2) + np.power(yAxis-inputLocations[i,1],2))/(scale_2))
		rowVector[i,0] = np.exp(-(np.multiply(xAxis-inputLocations[i,0],xAxis-inputLocations[i,0]) + np.multiply(yAxis-inputLocations[i,1],yAxis-inputLocations[i,1]))/(scale_2))

	#print rV == rowVector
	#print 'exit' 
	return rowVector		

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return np.sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

def distancetuple(p, q):
    """ compute Euclidean distance for 2D """
    (x1,y1) = p
    (x2,y2) = q
    return np.sqrt((x1-x2)**2+(y1-y2)**2)

def updateGoalPosition(Parameter,vorpos,xposcen,yposcen,xposcenobs,yposcenobs,mindis,movesampling):
	#mindis = 0.7
	numcoincide = 0
	coincidedflag = True
	movepercent = 0
	#movesampling = 10.0
	
	posinitx=[]
	posinity=[]

	for i in range(Parameter.NumberOfAgents): #vorpos = goal awal
		posinitx.append(vorpos[i,0])
		posinity.append(vorpos[i,1])
	ceninitx = xposcen # centroid asli
	ceninity = yposcen
	for i in range(Parameter.NumberOfAgents):
		if(xposcenobs[i] != None): # centroid baru
			ceninitx[i] = xposcenobs[i]
			ceninity[i] = yposcenobs[i]
	
	while (coincidedflag and movepercent < 1.001): # belum di coding untuk jika masuk ke obstacle
		movepercent = movepercent + 1.0/movesampling
		listOfCoincided = []
		for i in range(len(posinitx)):
			for j in range(i+1,len(posinity)):
				if(distance([posinitx[i],posinity[i]],[posinitx[j],posinity[j]]) < mindis):
					if i not in listOfCoincided:
						listOfCoincided.append(i)
					if j not in listOfCoincided:
						listOfCoincided.append(j)
					#print repr(i)+' and '+repr(j)+' coincide'
					numcoincide = numcoincide+1
		if(len(listOfCoincided) == 0):
			break;
		difx = []#(ceninitx-posinitx)*0.1
		dify = []#(ceninity-posinity)*0.1
		for i in range(len(posinitx)):
			if i in listOfCoincided:
				difx.append(posinitx[i]+(ceninitx[i]-posinitx[i])*movepercent)
				dify.append(posinity[i]+(ceninity[i]-posinity[i])*movepercent)
			else:
				difx.append(posinitx[i])
				dify.append(posinity[i])
				
	
		posinitx = difx
		posinity = dify
		#print numcoincide
		#print listOfCoincided
	return posinitx,posinity

def highsaturate(a,b,constrain):
	if(a >= constrain):
		a = constrain-1
	if(b >= constrain):
		b = constrain-1
	return a,b

def lowsaturate(a,b,constrain):
	if(a <= constrain):
		a = constrain
	if(b <= constrain):
		b = constrain
	return a,b

def highsaturatetuple(p,constrain):
	(x,y) = p
	if(x >= constrain):
		x = constrain
	if(y >= constrain):
		y = constrain
	return (x,y)

def lowsaturatetuple(p,constrain):
	(x,y) = p
	if(x <= constrain):
		x = constrain
	if(y <= constrain):
		y = constrain
	return (x,y)

def insidetheimage(i,j,p,q):
	if((i >= 0) and (i < p) and (j >= 0) and (j < q)):
		return True
	else:
		return False

def floodfill(M,i,j,lbl):
	if(insidetheimage(i,j,M.shape[0],M.shape[1]) and M[i,j]==1):
		M[i,j] = lbl
		M,lbl=floodfill(M,i+1,j,lbl)
		M,lbl=floodfill(M,i,j+1,lbl)
		M,lbl=floodfill(M,i,j-1,lbl)
		M,lbl=floodfill(M,i-1,j,lbl)
		M,lbl=floodfill(M,i+1,j-1,lbl)
		M,lbl=floodfill(M,i-1,j+1,lbl)
		M,lbl=floodfill(M,i-1,j-1,lbl)
		M,lbl=floodfill(M,i+1,j+1,lbl)
	return M,lbl

def countregion(M):
	temp = M.copy()
	m=2
	for i in range(temp.shape[0]):
		for j in range(temp.shape[1]):
			if(temp[i,j] == 1):
				temp,m = floodfill(temp,i,j,m)
				m=m+1
	return temp,m-2
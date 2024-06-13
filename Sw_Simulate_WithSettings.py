from __future__ import print_function
import numpy as np
import random
import matplotlib.pyplot as plt

import csv
class SW_Sim_WithSettings(object):




    # Goals achievable for p1 = .6; p2 = .4; s1 = 3; s2 = 4; s3 = 5;
    reliabilityGoal = 0.6
    performanceGoal = 5.0
    costGoal = 0.1
    # cost -> minimize

    # Control parameters
    p1 = .5;
    p2 = .5;
    serviceLevel = np.array([1.0, 1.0, 1.0])  # 1 to 5

    #Service settings
    reliability_settings = np.array([.9, .65, .45])
    performance_settings = np.array([2.0, 10.0, 20.0])
    cost_settings = np.array([15.0, 10.0, 5.0])


    # Num requests per time unit: large number -> better statistics
    # requestsPerTimeUnit = 10000

    # Variables for estimation
    numProcessedRequests    = np.array([0.0,0.0,0.0])
    numFailures             = np.array([0.0,0.0,0.0])
    cumulativeExecTime      = np.array([0.0,0.0,0.0])
    cumulativeCost          = np.array([0.0,0.0,0.0])

    def __init__(self):
        # Variables for estimation
        self.numProcessedRequests = np.array([0.0, 0.0, 0.0])
        self.numFailures = np.array([0.0, 0.0, 0.0])
        self.cumulativeExecTime = np.array([0.0, 0.0, 0.0])
        self.cumulativeCost = np.array([0.0, 0.0, 0.0])
        self.Setpoints = np.array([0.1, 0.6, 5.0])
        self.W = np.array([1.e-10, 100.0, 0.1])
        # Service settings
        self.reliability_settings = np.array([.9, .65, .45])
        self.performance_settings = np.array([2.0, 10.0, 20.0])
        self.cost_settings = np.array([15.0, 10.0, 5.0])

        self.GoalSpecification = [['CST', 0, 'MIN', self.cost_settings, self.Setpoints[0], self.W[0]],
                                  ['REL', 1, 'MAX', self.reliability_settings, self.Setpoints[1], self.W[1]],
                                  ['PER', 2, 'MIN', self.performance_settings, self.Setpoints[2], self.W[2]]]

    def get_GoalSpecification(self,ActiveGoals,Subject_Goal=''):
        index_CST=0
        index_REL=0
        index_PER=0
        if 'CST' in ActiveGoals:
            index_CST = 0
            if 'REL' in ActiveGoals:
                index_REL = 1
            if 'PER' in ActiveGoals:
                index_PER = 2
        else:
            if 'REL' in ActiveGoals:
                index_REL = 0
                if 'PER' in ActiveGoals:
                    index_PER = 1
            else:
                index_PER = 0
        self.GoalSpecification = [['CST', index_CST, 'MIN', self.cost_settings, self.Setpoints[0], self.W[0]],
                                  ['REL', index_REL, 'MAX', self.reliability_settings, self.Setpoints[1], self.W[1]],
                                  ['PER', index_PER, 'MIN', self.performance_settings, self.Setpoints[2], self.W[2]]]

        if  Subject_Goal=='':
            G=self.GoalSpecification
        else:
            G=[x for x in self.GoalSpecification if x[0]==Subject_Goal ]
        return G


    def callService(self,serviceNumber):
        # global numProcessedRequests, numFailures, cumulativeExecTime, cumulativeCost, serviceLevel
        # Service settings
        reliability_settings = self.reliability_settings    #np.array([.9, .65, .45])
        performance_settings = self.performance_settings    #np.array([2.0, 10.0, 20.0])
        cost_settings = self.cost_settings                  #np.array([15.0, 10.0, 5.0])

        # Simulating service execution
        execTime = max(
            random.expovariate(np.float(self.serviceLevel[serviceNumber]) ** 2 / np.float(performance_settings[serviceNumber])),
            0.0001)  # Avoid numerical zeros in the monitoring
        # execTime = float(performance_settings[serviceNumber])/(float(serviceLevel[serviceNumber]) ** 2)
        assert execTime > 0.0
        failure = 1.0 if random.random() > reliability_settings[serviceNumber] else 0.0
        cost = self.serviceLevel[serviceNumber] * cost_settings[serviceNumber]

        # Storing for statistics
        self.numProcessedRequests[serviceNumber] += 1
        self.numFailures[serviceNumber] += failure
        self.cumulativeExecTime[serviceNumber] += execTime
        self.cumulativeCost[serviceNumber] += cost

        # return numProcessedRequests,numFailures,cumulativeExecTime,cumulativeCost


    # for time in range(0, timeSteps):
    def SW_Model(self,Conf, DiffConf):
        # Variables for estimation
        

        self.p1 = (Conf.T)[0] + (DiffConf.T)[0]
        self. p2 = (Conf.T)[1] + (DiffConf.T)[1]
        self.serviceLevel[0] = (Conf.T)[2] + (DiffConf.T)[2]
        self.serviceLevel[1] = (Conf.T)[3] + (DiffConf.T)[3]
        self.serviceLevel[2] = (Conf.T)[4] + (DiffConf.T)[4]
        
        requestsPerTimeUnit = 20000
        
        # Execute services
        # At least one call per service
        self.callService(0)
        self.callService(1)
        self.callService(2)

        for request in range(0, requestsPerTimeUnit - 3):
            dice = random.random()
            if dice < self.p1:
                self.callService(0)
            else:
                dice = random.random()
                if dice < self.p2:
                    self.callService(1)
                else:
                    self.callService(2)

            # Local estimates
        measuredReliability = np.ones(3) - np.divide(1.0 * self.numFailures,
                                                     1.0 * self.numProcessedRequests)
        measuredExecTime = np.divide(1.0 * self.cumulativeExecTime,
                                     1.0 * self.numProcessedRequests)
        measuredCost = np.divide(1.0 * self.cumulativeCost,
                                 1.0 * self.numProcessedRequests)

        # Composed estimate -> measure
        globalMeasuredReliability = self.p1 * measuredReliability[0] + (1.0 - self.p1) * (
                self.p2 * measuredReliability[1] + (1.0 - self.p2) * measuredReliability[2])
        globalMeasuredExecTime = self.p1 * measuredExecTime[0] + (1.0 - self.p1) * (
                self.p2 * measuredExecTime[1] + (1.0 - self.p2) * measuredExecTime[2])
        globalMeasuredCost = self.p1 * measuredCost[0] + (1.0 - self.p1) * (self.p2 * measuredCost[1] + (1.0 - self.p2) * measuredCost[2])
        return globalMeasuredCost, globalMeasuredReliability, globalMeasuredExecTime
    def SW_Model2(self,Conf):

        self.p1 = (Conf.T)[0]
        self. p2 = (Conf.T)[1]
        
        self.serviceLevel[0] = np.float(Conf.T[2])
        self.serviceLevel[1] = np.float(Conf.T[3])
        self.serviceLevel[2] =np.float( Conf.T[4])
        
        requestsPerTimeUnit = 100000
        

        # Execute services
        # At least one call per service
        self.callService(0)
        self.callService(1)
        self.callService(2)

        for request in range(0, requestsPerTimeUnit - 3):
            dice = random.random()
            if dice < self.p1:
                self.callService(0)
            else:
                dice = random.random()
                if dice < self.p2:
                    self.callService(1)
                else:
                    self.callService(2)

            # Local estimates
        measuredReliability = np.ones(3) - np.divide(1.0 * self.numFailures,
                                                     1.0 * self.numProcessedRequests)
        measuredExecTime = np.divide(1.0 * self.cumulativeExecTime,
                                     1.0 * self.numProcessedRequests)
        measuredCost = np.divide(1.0 * self.cumulativeCost,
                                 1.0 * self.numProcessedRequests)

        # Composed estimate -> measure
        globalMeasuredReliability = self.p1 * measuredReliability[0] + (1.0 - self.p1) * (
                self.p2 * measuredReliability[1] + (1.0 - self.p2) * measuredReliability[2])
        globalMeasuredExecTime = self.p1 * measuredExecTime[0] + (1.0 - self.p1) * (
                self.p2 * measuredExecTime[1] + (1.0 - self.p2) * measuredExecTime[2])
        globalMeasuredCost = self.p1 * measuredCost[0] + (1.0 - self.p1) * (self.p2 * measuredCost[1] + (1.0 - self.p2) * measuredCost[2])
        return np.array([globalMeasuredCost, globalMeasuredReliability, globalMeasuredExecTime])



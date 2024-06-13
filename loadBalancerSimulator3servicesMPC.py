from __future__ import print_function
import numpy as np
import random
import matplotlib.pyplot as plt
import libs.mpyc as reg
import libs.utils as ut
from libs.init_mpc import initialize_mpc
import itertools
import csv

Services_Plot=[]
#Goals achievable for p1 = .6; p2 = .4; s1 = 3; s2 = 4; s3 = 5;
reliabilityGoal = 0.7  # 0.6
performanceGoal = 4.0   # 5.0
costGoal = 0.1
#cost -> minimize

#Control parameters
p1 = .5;
p2 = .5;
serviceLevel = np.array([1.0,1.0,1.0]) # 1 to 5

#Service settings
reliability_settings = np.array([.9, .65, .45])
performance_settings = np.array([2.0, 10.0, 20.0])
cost_settings = np.array([15.0, 10.0, 5.0])

#Num requests per time unit: large number -> better statistics
requestsPerTimeUnit = 10000

#Variables for estimation
numProcessedRequests    = np.array([0.0,0.0,0.0])
numFailures             = np.array([0.0,0.0,0.0])
cumulativeExecTime      = np.array([0.0,0.0,0.0])
cumulativeCost          = np.array([0.0,0.0,0.0])



def callService(serviceNumber):
    global numProcessedRequests, numFailures, cumulativeExecTime, cumulativeCost, serviceLevel

    #Simulating service execution
    execTime = max(random.expovariate(float(serviceLevel[serviceNumber])**2/float(performance_settings[serviceNumber])), 0.0001) # Avoid numerical zeros in the monitoring
    #execTime = float(performance_settings[serviceNumber])/(float(serviceLevel[serviceNumber]) ** 2)
    assert execTime>0.0
    failure =  1.0 if random.random() > reliability_settings[serviceNumber] else 0.0
    cost = serviceLevel[serviceNumber]*cost_settings[serviceNumber]

    #Storing for statistics
    numProcessedRequests[serviceNumber]+=1
    numFailures[serviceNumber]+=failure
    cumulativeExecTime[serviceNumber] += execTime
    cumulativeCost[serviceNumber]+=cost


def simulate(timeSteps, tau, filename='icse17_trace'):
    global numProcessedRequests, numFailures, cumulativeExecTime, cumulativeCost, p1, p2, serviceLevel, reliability_settings
    global reliabilityGoal, performanceGoal, costGoal

    reliabilityGoalHistory = []
    performanceGoalHistory = []
    costGoalHistory = []
    reliabilityMeasuredHistory = []
    performanceMeasuredHistory = []
    costMeasuredHistory = []

    # Initialize matrices for MPC
    A,B,C,D,L,Q,R,Umin,Umax,DeltaUmin,DeltaUmax,Qn,Rn,Lk,Pk,sp = initialize_mpc();
    #Initializing the controller
    controller = reg.MPCController(A, B, C, D,\
                            L, Q, R,\
                            Lk, Pk, Qn, Rn,\
                            Umin, Umax, DeltaUmin, DeltaUmax,\
                            optim=1,\
                            fast=0,\
                            time_varying=1)

    initial_u = np.matrix([[p1],[p2],[serviceLevel[0]],[serviceLevel[1]],[serviceLevel[2]]])
    controller.initialize_controller_state(initial_u)

    with open(filename+'.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        csvwriter.writerow(['time', 'rel_goal', 'rel_err', 'perf_goal', 'perf_err', 'cost_goal', 'cost_err'])


    with open(filename+'_control.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        csvwriter.writerow(['time','p1','p2','l1','l2','l3'])

    avg_reliability_tau = 0.0
    avg_performance_tau = 0.0
    avg_cost_tau = 0.0

    for time in range(0, timeSteps):

        #Store old previous state
        numProcessedRequestsOld   = np.array(numProcessedRequests)
        numFailuresOld            = np.array(numFailures)
        cumulativeExecTimeOld     = np.array(cumulativeExecTime)
        cumulativeCostOld         = np.array(cumulativeCost)

        #Execute services
        #At least one call per service
        callService(0)
        callService(1)
        callService(2)

        for request in range(0,requestsPerTimeUnit-3):
            dice = random.random()
            if dice<p1 :
                callService(0)
            else:
                dice = random.random()
                if dice<p2:
                    callService(1)
                else:
                    callService(2)


        #measuredReliability = reliability_settings
        #measuredExecTime = np.divide(1.0,np.divide(performance_settings,np.power(serviceLevel,2)))
        #measuredCost = cost_settings*serviceLevel

        #Local estimates
        measuredReliability = np.ones(3) - np.divide( 1.0*numFailures - numFailuresOld, 1.0*numProcessedRequests - numProcessedRequestsOld)
        measuredExecTime = np.divide(1.0*cumulativeExecTime - cumulativeExecTimeOld, 1.0*numProcessedRequests - numProcessedRequestsOld)
        measuredCost = np.divide(1.0*cumulativeCost - cumulativeCostOld, 1.0*numProcessedRequests - numProcessedRequestsOld)

        #Composed estimate -> measure
        globalMeasuredReliability = p1*measuredReliability[0] +(1.0-p1)*(p2*measuredReliability[1] + (1.0-p2)*measuredReliability[2])
        globalMeasuredExecTime = p1*measuredExecTime[0] +(1.0-p1)*(p2*measuredExecTime[1] + (1.0-p2)*measuredExecTime[2])
        globalMeasuredCost = p1*measuredCost[0] +(1.0-p1)*(p2*measuredCost[1] + (1.0-p2)*measuredCost[2])

        #PWM measures
        avg_reliability_tau += float(globalMeasuredReliability)
        avg_performance_tau += float(globalMeasuredExecTime)
        avg_cost_tau += float(globalMeasuredCost)

        # Decide setpoints (from FSE paper)

        # if time==100*tau:
        #     reliabilityGoal = 0.9
            #performanceGoal = 1.0
        if time == 100 * tau:
            reliabilityGoal = 0.6

        if time==200*tau:
            # Change the reliability of Service2 to 0.6
            reliability_settings = np.array([0.9, 0.55, 0.45])

            # controller.Umin=np.matrix([[0.9],[0.9],[1.0],[1.0],[1.0]]);
            # Change the reliability of Service2 to 0.6
            #reliability_settings = np.array([0.8, 0.5, 0.45])
        if time == 300 * tau:

            # S1,S3 are active and S1 fails
            controller.Umax = np.matrix([[1.0], [0.0001], [5.0], [5.0], [5.0]])

        # if time==100 * tau:
        #     # controller.Umax = np.matrix([[1.0], [1.0], [5.0], [5.0], [5.0]])
        #     #controller.Umin=np.matrix([[0.0],[0.0],[1.0],[1.0],[1.0]]);
        #     reliability_settings = np.array([0.8, 0.5, 0.4]) #np.array([.9, .65, .45])
        #     performance_settings = np.array([5.0, 12.0, 22.0])  #np.array([2.0, 10.0, 20.0])
        #     cost_settings = np.array([20.0, 15.0, 8.0]) # np.array([15.0, 10.0, 5.0])

        if time == 3300*tau:
            reliabilityGoal = 0.5

        if time == 4200*tau:
            reliabilityGoal = 0.3

        if time == 5200*tau:
            performanceGoal = 1.0

        if time == 6500*tau:
            reliabilityGoal = .8


        if time%tau==0: # Decide control every tau timesteps
            avg_reliability_tau = avg_reliability_tau/tau if time>0 else avg_reliability_tau
            avg_performance_tau = avg_performance_tau/tau if time>0 else avg_performance_tau
            avg_cost_tau = avg_cost_tau/tau if time>0 else avg_cost_tau

            #Errors
            errorReliability = reliabilityGoal - avg_reliability_tau
            errorPerformance = performanceGoal - avg_performance_tau
            errorCost = costGoal - avg_cost_tau

            #Control
            sp = np.matrix([[costGoal],[reliabilityGoal],[performanceGoal]])
            yy = np.matrix([[avg_cost_tau],[avg_reliability_tau],[avg_performance_tau]])
            uu = controller.compute_u(yy,sp)

            plan = []

            l1_over = min(max(np.ceil(uu.item(2)),1),5)
            l1_under = min(max(np.floor(uu.item(2)),1),5)
            t1_over = int(np.round((uu.item(2) - l1_under) / (l1_over-l1_under))) if l1_over != l1_under else int(tau)
            t1_under = int(tau)-t1_over
            l1_plan = [l1_over]*t1_over + [l1_under]*t1_under

            l2_over = min(max(np.ceil(uu.item(3)),1),5)
            l2_under = min(max(np.floor(uu.item(3)),1),5)
            t2_over = int(np.round((uu.item(3) - l2_under) / (l2_over - l2_under))) if l2_over != l2_under else int(tau)
            t2_under = int(tau) - t2_over
            l2_plan = [l2_over] * t2_over + [l2_under] * t2_under

            l3_over = min(max(np.ceil(uu.item(4)),1),5)
            l3_under = min(max(np.floor(uu.item(4)),1),5)
            t3_over = int(np.round((uu.item(4) - l3_under) / (l3_over - l3_under))) if l3_over != l3_under else int(tau)
            t3_under = int(tau) - t3_over
            l3_plan = [l3_over] * t3_over + [l3_under] * t3_under

            p1_plan = [uu.item(0)] * int(tau)
            p2_plan = [uu.item(1)] * int(tau)

            for i in range(int(tau)):
                plan.append((p1_plan[i],p2_plan[i],l1_plan[i],l2_plan[i],l3_plan[i]))


            # Store data
            reliabilityGoalHistory.append(reliabilityGoal)
            performanceGoalHistory.append(performanceGoal)
            costGoalHistory.append(costGoal)
            reliabilityMeasuredHistory.append(avg_reliability_tau)
            performanceMeasuredHistory.append(avg_performance_tau)
            costMeasuredHistory.append(avg_cost_tau)

            with open(filename+'.csv', 'a') as csvfile:
                csvwriter = csv.writer(csvfile, delimiter=',',
                                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
                csvline = []
                csvline.append(time)
                csvline.append(reliabilityGoal)
                csvline.append(reliabilityGoal - avg_reliability_tau)
                csvline.append(performanceGoal)
                csvline.append(performanceGoal - avg_performance_tau)
                csvline.append(costGoal)
                csvline.append(costGoal - avg_cost_tau)
                csvline = [str(val) for val in csvline]
                csvwriter.writerow(csvline)


            # Reset accumulators
            avg_reliability_tau = 0.0
            avg_performance_tau = 0.0
            avg_cost_tau = 0.0

        conf = plan.pop()
        # TODO serviceLevel should be rounded
        p1 = conf[0]
        p2 = conf[1]
        serviceLevel[0] = conf[2]
        serviceLevel[1] = conf[3]
        serviceLevel[2] = conf[4]

        with open(filename + '_control.csv', 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',',
                                   quotechar='|', quoting=csv.QUOTE_MINIMAL)
            csvwriter.writerow([str(time), str(p1), str(p2), str(serviceLevel[0]),str(serviceLevel[1]), str(serviceLevel[2])])
            print([str(time), str(p1), str(p2), str(serviceLevel[0]),str(serviceLevel[1]), str(serviceLevel[2])])
            if time % tau == 0:
                Services_Plot.insert(len(Services_Plot),
                                 [p1 * 100.0, p2 * 100.0, ((1.0 - p1) * (1.0 - p2)) * 100.0])


        #initial_u = np.matrix([[p1],[p2],[serviceLevel[0]],[serviceLevel[1]],[serviceLevel[2]]])
        #controller.initialize_controller_state(initial_u)



        print('p1 = '+str(p1)+' uu0 = '+str(uu.item(0)))
        print('p2 = '+str(p2)+' uu1 = '+str(uu.item(1)))
        print('sl1 = '+str(serviceLevel[0])+' uu2 = '+str(uu.item(2)))
        print('sl2 = '+str(serviceLevel[1])+' uu3 = '+str(uu.item(3)))
        print('sl3 = '+str(serviceLevel[2])+' uu4 = '+str(uu.item(4)))

        print('time: '+str(time)+'\tCost_tau: '+str('{0:.7f}'.format(avg_cost_tau))+'\tReliability_tau: '+str('{0:.7f}'.format(avg_reliability_tau))+'\tPerformance_tau: '+str('{0:.7f}'.format(avg_performance_tau)))
        print('time: ' + str(time) + '\tCost_global: ' + str('{0:.7f}'.format(globalMeasuredCost)) + '\tReliability_global: ' + str('{0:.7f}'.format(globalMeasuredReliability)) + '\tPerformance_global: ' + str('{0:.7f}'.format(globalMeasuredExecTime)))
        print('time: ' + str(time) + '\terrorCost: ' + str('{0:.7f}'.format(errorCost)) + '\terrorReliability: ' + str('{0:.7f}'.format(errorReliability)) + '\terrorPerformance: ' + str('{0:.7f}'.format(errorPerformance)))
        print('\t\t\tp1: '+str('{0:.5f}'.format(p1))+'\tp2: '+str('{0:.5f}'.format(p2))+'\ts1l: '+str('{0:.1f}'.format(serviceLevel[0]))+'\ts2l: '+str('{0:.1f}'.format(serviceLevel[1]))+'\ts3l: '+str('{0:.1f}'.format(serviceLevel[2])))


    try:
        #time0 = np.linspace(0.0, len(reliabilityGoalHistory)-1, len(reliabilityGoalHistory))
        # fig=plt.subplots(nrows=3,ncols=1)
        # fig.tight_layout(pad=3.0)
        AMOCS = plt.figure("AMOCS-MA")
        Reliability = plt.figure("Reliability_AMOCS-MA")
        #Reliability.tight_layout()
        #plt.subplot(3, 1, 1)
        # plt.plot(time0, reliabilityMeasuredHistory, 'k.-')
        # plt.plot(time0, reliabilityGoalHistory, 'r.-')
        plt.plot( reliabilityMeasuredHistory  , 'k.-',label ='Measured value')
        plt.plot( reliabilityGoalHistory  , 'r.-', label='Goal')
        #plt.xlabel('time (steps)')
        plt.ylabel('Reliability')
        plt.legend(borderaxespad=0,  loc='upper right' ,bbox_to_anchor=(1.2,1), fontsize="x-small") #,bbox_to_anchor=(1.2,1)
        plt.subplots_adjust(right=0.85)
        Reliability.tight_layout()
        Reliability.savefig('results/AMOCS-MA/Reliability.png')

        
        Performance = plt.figure("Performance_AMOCS-MA")
        Performance.tight_layout()
        plt.plot( performanceMeasuredHistory, 'k.-', label='Performance')
        plt.plot( performanceGoalHistory, 'r.-' , label='Goal')
        #plt.xlabel('time (steps)')
        plt.ylabel('Performance')
        
        Performance.savefig('results/AMOCS-MA/Performance.png')
        
        Cost = plt.figure("Cost_AMOCS-MA")
        Cost.tight_layout()
        plt.plot( costMeasuredHistory, 'k.-' , label ='Cost')
        
        plt.xlabel('time (steps)')
        plt.ylabel('Cost')
        Cost.savefig('results/AMOCS-MA/Cost.png')
        


        Sense = plt.figure("Service Probability_AMOCS-MA")
        Sense.tight_layout()
        plt.plot(list(zip(*Services_Plot))[0], color='red', linestyle='solid', label='Service 1')
        plt.plot(list(zip(*Services_Plot))[1], color='blue', linestyle='dashed', label='Service 2')
        plt.plot(list(zip(*Services_Plot))[2], color='lime', linestyle=(0, (10, 2, 2, 2, 2, 2)), label='Service 3')
        
        plt.xlabel('time (steps)')
        plt.legend( borderaxespad=0,bbox_to_anchor=(1.2,1) ,loc='upper right', fontsize="x-small")
                   
        plt.ylabel('Service selection')
        plt.subplots_adjust(right=0.85)
        Sense.savefig('results/AMOCS-MA/Service selection.png')

        


        plt.show()
    except:
        print("Cannot find $DISPLAY. The plot will cannot be visualized in this run. All the data in csv are availbe into:\n")
        print(" - Execution traces: "+str(filename)+'.csv\n')
        print(" - Control input traces: "+str(filename)+'_control.csv\n')

def saturation(x,uMin,uMax):
    k = 1e3;
    f = np.exp((x/((uMax - uMin)/2) - (uMax + uMin))*np.log10(k));
    y = uMin + (uMax - uMin) / 2*(1 + np.log10(np.sqrt(1 + (f*k)**2)/np.sqrt(1 + (f/k)**2)/k)/np.log10(k));
    return y




def get_systematic_extreme_confs(numRepetitions,shuffle=True):
    confs_to_explore = []
    p_grid = [0.0, 1.0]
    l_grid = range(1, 6)
    for conf in itertools.product(p_grid, p_grid, l_grid, l_grid, l_grid):
        for repetition in range(numRepetitions):
            confs_to_explore.append(conf)
    if shuffle:
        random.shuffle(confs_to_explore)
    return confs_to_explore


def get_systematic_grid_confs(numRepetitions,shuffle=True):
    confs_to_explore = []
    p_grid = [k * 0.05 for k in range(0, 21)]
    l_grid = range(1, 6)
    for conf in itertools.product(p_grid, p_grid, l_grid, l_grid, l_grid):
        for repetition in range(numRepetitions):
            confs_to_explore.append(conf)
    if shuffle:
        random.shuffle(confs_to_explore)
    return confs_to_explore


def get_random_extreme_confs(numConfs):
    confs_to_explore=[]
    for _ in range(len(numConfs)):
        p1 = 1.0 if random.random()<.5 else 0.0;
        p2 = 1.0 if random.random()<.5 else 0.0;
        serviceLevel1 = 5.0 if random.random()<.5 else 1.0;
        serviceLevel2 = 5.0 if random.random()<.5 else 1.0;
        serviceLevel3 = 5.0 if random.random()<.5 else 1.0;
        confs_to_explore.append((p1,p2,serviceLevel1,serviceLevel2,serviceLevel3))
    return confs_to_explore



def simulateLearning(fileName):
    global numProcessedRequests, numFailures, cumulativeExecTime, cumulativeCost, p1, p2, serviceLevel

    #Print header
    with open(fileName, 'w') as fOut:
        fOut.write('p1 p2 service1Level service2Level service3Level cost reliability performance\n')



    #confs_to_explore = get_systematic_extreme_confs(5,shuffle=True)
    confs_to_enforce = get_random_extreme_confs(5000)
    #confs_to_enforce = get_random_confs(50000,window=40,shuffle=True)
    #confs_to_explore = get_systematic_grid_confs(2)

    timeSteps = len(confs_to_enforce)

    for time in range(0, timeSteps):
        # print(str((time+1)*100.0/timeSteps)+'% completed')
        ut.progress((time+1),timeSteps)
        #Store old previous state
        numProcessedRequestsOld   = np.array(numProcessedRequests)
        numFailuresOld            = np.array(numFailures)
        cumulativeExecTimeOld     = np.array(cumulativeExecTime)
        cumulativeCostOld         = np.array(cumulativeCost)

        #Execute services
        #At least one call per service
        callService(0)
        callService(1)
        callService(2)

        for request in range(0,requestsPerTimeUnit-3):
            dice = random.random()
            if dice<p1 :
                callService(0)
            else:
                dice = random.random()
                if dice<p2:
                    callService(1)
                else:
                    callService(2)


        #measuredReliability = reliability_settings
        #measuredExecTime = np.divide(1.0,np.divide(performance_settings,np.power(serviceLevel,2)))
        #measuredCost = cost_settings*serviceLevel

        #Local estimates
        measuredReliability = np.ones(3) - np.divide( 1.0*numFailures - numFailuresOld, 1.0*numProcessedRequests - numProcessedRequestsOld)
        measuredExecTime = np.divide(1.0*cumulativeExecTime - cumulativeExecTimeOld, 1.0*numProcessedRequests - numProcessedRequestsOld)
        measuredCost = np.divide(1.0*cumulativeCost - cumulativeCostOld, 1.0*numProcessedRequests - numProcessedRequestsOld)

        #Composed estimate -> measure
        globalMeasuredReliability = p1*measuredReliability[0] +(1.0-p1)*(p2*measuredReliability[1] + (1.0-p2)*measuredReliability[2])
        globalMeasuredExecTime = p1*measuredExecTime[0] +(1.0-p1)*(p2*measuredExecTime[1] + (1.0-p2)*measuredExecTime[2])
        globalMeasuredCost = p1*measuredCost[0] +(1.0-p1)*(p2*measuredCost[1] + (1.0-p2)*measuredCost[2])

        #Errors
        errorReliability = reliabilityGoal - globalMeasuredReliability
        errorPerformance = performanceGoal - globalMeasuredExecTime
        errorCost = costGoal - globalMeasuredCost


        

        conf = confs_to_enforce.pop()
        p1 = conf[0]
        p2 = conf[1]
        serviceLevel[0] = conf[2]
        serviceLevel[1] = conf[3]
        serviceLevel[2] = conf[4]



        with open(fileName, 'a') as fOut:
            fOut.write(str(p1)+' '+str(p2)+' '+str(serviceLevel[0])+' '+str(serviceLevel[1])+' '+str(serviceLevel[2])+' '+str(globalMeasuredCost)+' '+str(globalMeasuredReliability)+' '+str(globalMeasuredExecTime)+'\n')
    print('\n')



if __name__ == "__main__":
    #Control
    tau = 3.0
    simulate(400*int(tau), tau) #800

    ##Generate trace file
    #simulateLearning('dataSimBinder.csv')

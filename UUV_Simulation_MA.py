import numpy as np
import random


class UUV_Sim_MA(object):
    # Number of Sensors
    num_sensors = 5
    ####### vector to show failure or not
    Vec_fail = np.ones(num_sensors)
    ServiceMonitor = []


    # Sensors Settings

    EnergyCons_Settings = np.array([170, 135, 118, 100, 78])
    ScanSpeed_Settings= np.array([2.6, 3.6, 2.6, 3.0, 3.6])
    Accuracy_Settings= np.array([0.97, 0.89, 0.83, 0.74, 0.49])

    # Variables for estimation
    NumCallSensors = np.zeros(num_sensors)
    cumulativeScannedSurface = np.zeros(num_sensors)
    cumulativeTimeElapsed = np.zeros(num_sensors)
    cumulativeEnergyConsumption = np.zeros(num_sensors)
    cumulativeNumFailures = np.zeros(num_sensors)


    def __init__(self):
        self.Setpoints = np.array([1.0, 2.7, 150.0]) 
        self.Weights = np.array([0.01, 100.0, 0.1])
        self.NumCallSensors = np.zeros(self.num_sensors)
        self.cumulativeScannedSurface = np.zeros(self.num_sensors)
        self.cumulativeTimeElapsed = np.zeros(self.num_sensors)
        self.cumulativeEnergyConsumption = np.zeros(self.num_sensors)
        self.cumulativeNumFailures = np.zeros(self.num_sensors)
        self.GoalSpecification=[['ACC',0,'MAX',self.Accuracy_Settings,self.Setpoints[0],0.01],['SS',1,'MAX',self.ScanSpeed_Settings,self.Setpoints[1],100.0],['EC',2,'MIN',self.EnergyCons_Settings,self.Setpoints[2],0.1]]

    def get_GoalSpecification(self,ActiveGoals,Subject_Goal=''):
        index_ACC=0
        index_SS=0
        index_EC=0
        if 'ACC' in ActiveGoals:
            index_ACC = 0
            if 'SS' in ActiveGoals:
                index_SS = 1
            if 'EC' in ActiveGoals:
                index_EC = 2
        else:
            if 'SS' in ActiveGoals:
                index_SS = 0
                if 'EC' in ActiveGoals:
                    index_EC = 1
            else:
                index_EC = 0
        self.GoalSpecification = [['ACC', index_ACC, 'MAX', self.Accuracy_Settings, self.Setpoints[0], 0.01],
                                  ['SS', index_SS, 'MAX', self.ScanSpeed_Settings, self.Setpoints[1], 100.0],
                                  ['EC', index_EC, 'MIN', self.EnergyCons_Settings, self.Setpoints[2], 0.1]]

        if  Subject_Goal=='':
            G=self.GoalSpecification
        else:
            G=[x for x in self.GoalSpecification if x[0]==Subject_Goal ]
        return G

    def get_setpoint(self,Subject_Goal=''):
        if Subject_Goal=='ACC':
            return self.Setpoints[0]
        if Subject_Goal=='SS':
            return self.Setpoints[1]
        if Subject_Goal=='EC':
            return self.Setpoints[2]
        if Subject_Goal=='':
            return self.Setpoints


    def get_Weight(self, Subject_Goal=''):
        if Subject_Goal == 'ACC':
            return self.Weights[0]
        if Subject_Goal == 'SS':
            return self.Weights[1]
        if Subject_Goal == 'EC':
            return self.Weights[2]
        if Subject_Goal == '':
            return self.Weights

    def CallSensors(self,U):
        for i in range(self.num_sensors):
            if U[i]>0.0 :
                failure = 1.0 if random.random() > self.Accuracy_Settings[i] else 0.0
                self.NumCallSensors[i] +=1
                self.cumulativeScannedSurface[i] += U[i] * self.ScanSpeed_Settings[i]
                self.cumulativeTimeElapsed[i] += U[i]
                self.cumulativeEnergyConsumption[i] += U[i] * self.EnergyCons_Settings[i]
                self.cumulativeNumFailures[i] += failure

    def UUV_Run(self,U):
        Num_Runs=100
        for k in range(Num_Runs):
            self.CallSensors(U)
        #Local Estimation

        MeasuredAccurecy=np.ones(self.num_sensors) - np.divide(1.0 * self.cumulativeNumFailures, 1.0 * self.NumCallSensors,out=np.zeros_like(self.cumulativeNumFailures), where=self.NumCallSensors!=0.0)

        ######## Service Monitor
        ScannedSurface_Sensors = self.Vec_fail * np.array([self.cumulativeScannedSurface[i] /
                                                           self.cumulativeTimeElapsed[i] if self.cumulativeTimeElapsed[
                                                                                                i] != 0 else 0.0 for i
                                                           in range(len(U))])
        MeasuredEnergeyCons_Sensors = self.Vec_fail * \
                                      np.array([self.cumulativeEnergyConsumption[i] / self.NumCallSensors[i] if
                                                self.NumCallSensors[i] != 0.0 else 0.0 for i in range(len(U))])
        MeasuredAccurecy_Sensors = self.Vec_fail * self.Accuracy_Settings
        self.ServiceMonitor = [[MeasuredAccurecy_Sensors[i], ScannedSurface_Sensors[i], MeasuredEnergeyCons_Sensors[i]]
                               for i in range(len(U))]

        #Global Measurements
        
        # GlobalMeasuredAccuracy = U.T.dot(self.Accuracy_Settings.reshape(self.num_sensors, 1)).squeeze()
        # GlobalScannedSurface=np.sum(self.cumulativeScannedSurface)
        # GlobalTimeElapsed=np.sum(self.cumulativeTimeElapsed)
        # GlobalMeasuredSpeed= GlobalScannedSurface / GlobalTimeElapsed
        # GlobalMeasuredEnergeyCons=np.sum(self.cumulativeEnergyConsumption) / (1.0 * Num_Runs)
        GlobalMeasuredAccuracy = (self.Vec_fail * U.squeeze()).T.dot(
            self.Accuracy_Settings.reshape(self.num_sensors, 1)).squeeze()
        GlobalScannedSurface = np.sum(self.Vec_fail * self.cumulativeScannedSurface)
        GlobalTimeElapsed = np.sum(self.Vec_fail * self.cumulativeTimeElapsed)
        GlobalMeasuredSpeed = GlobalScannedSurface / GlobalTimeElapsed
        GlobalMeasuredEnergeyCons = np.sum(self.Vec_fail * self.cumulativeEnergyConsumption) / (1.0 * Num_Runs)

        return np.array([GlobalMeasuredAccuracy,GlobalMeasuredSpeed,GlobalMeasuredEnergeyCons])

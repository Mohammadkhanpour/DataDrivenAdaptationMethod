import numpy as np
import random

class TAS_Sim_System_MA(object):
    # Call Drug Service
    DrugService_Call=1.0
    DrugService_per= 0.75
    AlarmService_per=0.25
    Vec_prob= np.array([1.0,1.0,1.0,1.0,1.0,0.25,0.25,0.25,0.75])
    # Alarm Service
    # if DrugService_Call == 1.0 :
    #     U_AS_Call=np.zeros(3)
    # Probability of Panic
    p_panic=1.0
    p_DS=0.9
    # Number of Alternative Services
    num_AS = 3
    num_MAS = 5
    num_Us= num_MAS+num_AS
    num_DS=1
    num_services=9
    # Failure Vector
    Vec_fail = np.ones(num_services)
    # Monitoring
    ServiceMonitor = []
    # TAS Variable
    TAS_Failure_Flag=False
    TAS_Failure_Number=0
    # Zero Actuators for Services
    U_Zero_Alarm=np.zeros(num_AS)
    U_Zero_Medical = np.zeros(num_MAS)
    U_Zero_Drug = np.zeros(num_DS)
    # TAS Services Specifications

    Cost_Settings = np.array([9.8,8.9,9.3,7.3,11.9,4.1,2.5,6.8,0.1])
    FR_Settings= np.array([0.06, 0.1,0.15,0.25,0.05,0.3,0.4,0.08,0.12])
    ResponseTime_Settings=np.array([22.0, 27.0, 31.0,29.0,20.0,11.0,9.0,3.0,1.0])
    ###########################
    # Variables for estimation
    NumCallServices = np.zeros(num_services)
    cumulative_Cost = np.zeros(num_services)
    cumulative_ResponseTime = np.zeros(num_services)
    cumulative_NumberFailures = np.zeros(num_services)
    cumulative_FailureRefernces = np.zeros(num_services)
    ######################
    Monitor_COST_TAS= np.zeros(num_services)
    Monitor_ResponseTime_TAS=np.zeros(num_services)
    Monitor_Failure_TAS=np.zeros(num_services)
    ##########################
    GlobalMeasured_Cost_TAS=0.0
    GlobalMeasured_ResponseTime_TAS=0.0
    GlobalMeasured_FR= 0.0
    GlobalFailureRate=0.0
    GlobalFailureReference=0.0

    ############################
    def __init__(self):
        ##
        # pick = random.randint(1, 3)
        # if pick == 3:
        #     self.DrugService_Call=0.0
        # else:
        #     self.DrugService_Call = 1.0



        # Variables for estimation
        # Monitoring
        self.ServiceMonitor = []
        self. NumCallServices = np.zeros(self.num_services)
        self.cumulative_Cost = np.zeros(self.num_services)
        self.cumulative_ResponseTime = np.zeros(self.num_services)
        self.cumulative_NumberFailures = np.zeros(self.num_services)
        self.cumulative_FailureRefernces = np.zeros(self.num_services)
        # TAS Variable
        self.TAS_Failure_Flag = False
        self.TAS_Failure_Number = 0
        # Zero Actuators for Services
        self.U_Zero_Alarm = np.zeros(self.num_AS)
        self.U_Zero_Medical = np.zeros(self.num_MAS)
        self.U_Zero_Drug = np.zeros(self.num_DS)

    def __init__(self,num_srv=9,num_alarm=3,num_ms=5,num_ds=1):
        # Number services
        self.num_MAS=num_ms
        self.num_AS=num_alarm
        self.num_DS=num_ds
        self.num_services=num_srv
        # Variables for estimation
        self.NumCallServices = np.zeros(num_srv)
        self.cumulative_Cost = np.zeros(num_srv)
        self.cumulative_ResponseTime = np.zeros(num_srv)
        self.cumulative_NumberFailures = np.zeros(num_srv)
        self.cumulative_FailureRefernces = np.zeros(num_srv)
        # TAS Variable
        self.TAS_Failure_Flag = False
        self.TAS_Failure_Number = 0
        # Zero Actuators for Services
        self.U_Zero_Alarm = np.zeros(num_alarm)
        self.U_Zero_Medical = np.zeros(num_ms)
        self.U_Zero_Drug = np.zeros(num_ds)

   

    def CallService(self, U):
        for i in range(len(U)):
            if U[i] > 0.0:
                self.NumCallServices[i] += 1
                self.cumulative_Cost[i] += U[i] * self.Cost_Settings[i]  # U[i] *
                self.cumulative_FailureRefernces[i] += U[i] * self.FR_Settings[i]  # U[i] *
                self.cumulative_ResponseTime[i] += U[i] * self.ResponseTime_Settings[i]  # U[i] *
                # failure = 1.0 if random.random() <= self.FR_Settings[i] else 0.0
                # if failure==1.0 :
                #     self.cumulative_NumberFailures[i] += failure
                #     self.TAS_Failure_Flag=True
                #
                # else:
                #     self.cumulative_Cost[i] += U[i] * self.Cost_Settings[i] # U[i] *
                #     self.cumulative_FailureRefernces[i] += U[i] * self.FR_Settings[i] # U[i] *
                #     self.cumulative_ResponseTime[i] += U[i] * self.ResponseTime_Settings[i] # U[i] *


    def TAS_Run2(self, U):
        self.TAS_Failure_Flag=False
        self.TAS_Failure_Number = 0
        # U is Actuator
        U = U.squeeze()
        Y = np.array([1.0])  # only One Drug Service
        U = np.concatenate([U, Y])
        print("U:",U, U.shape)
        #############################
        U_medical=np.concatenate([U[0:5],self.U_Zero_Alarm,self.U_Zero_Drug])
        U_alarm=np.concatenate([self.U_Zero_Medical,U[5:8],self.U_Zero_Drug])
        U_drug=np.concatenate([self.U_Zero_Medical,self.U_Zero_Alarm,U[8:9]])
        print("U_medical:",U_medical, len(U_medical))
        print("U_alarm:",U_alarm , len(U_alarm))
        print("U_drug:",U_drug , len(U_drug))
        ##################################
        Num_Runs = 100
        for k in range(Num_Runs):
            p_callservice=random.random()
            # CallService
            if p_callservice<=self.p_panic :
                self.CallService(U_medical)
                pick = random.randint(1,3)   
                #if not self.TAS_Failure_Flag:
                if pick == 3:
                    self.CallService(U_alarm)
                else:
                    self.CallService(U_drug)
                
            else:
                self.CallService(U_alarm)



        self.GlobalMeasured_Cost_TAS = np.sum(self.cumulative_Cost) / (1.0 * Num_Runs)
        self.GlobalMeasured_ResponseTime_TAS = np.sum(self.cumulative_ResponseTime) / (1.0 * Num_Runs)
        self.GlobalMeasured_FR = (1.0 * self.TAS_Failure_Number) / (1.0 * Num_Runs)  
        self.GlobalFailureRate=  U.T.dot(self.FR_Settings.reshape(self.num_services, 1)).squeeze()    
        self.GlobalFailureReference= np.sum(self.cumulative_FailureRefernces)/(1.0 * Num_Runs)

        return np.array([self.GlobalMeasured_Cost_TAS, self.GlobalMeasured_ResponseTime_TAS,self.GlobalFailureRate])

    def TAS_Run(self,U):
        # U is Actuator
        U = U.squeeze()
        U_AS_Zero= np.array([0.0, 0.0, 0.0])
        Y = np.array([1.0])  # only One Drug Service
        # U_DS = np.concatenate([U[0:5],U_AS_Zero, Y])
        U_DS = np.concatenate([U, Y])
        Z = np.array([0.0])  # only One Drug Service
        U_AS = np.concatenate([U, Z])
        Num_Runs = 10000#20000
        for k in range(Num_Runs):
            U = U_DS
            self.CallService(U)
            # pick = random.randint(1, 3)
            # p_callservice = random.random()
            # if  self.DrugService_Call== 1.0 : #pick != 3:  # p_callservice<=self.p_DS:
            #     U=U_DS
            #     self.CallService(U)
            # else:
            #     U=U_AS
            #     self.CallService(U)


        self.GlobalFailureReference = (np.sum(self.Vec_fail * self.Vec_prob * self.cumulative_FailureRefernces) / (1.0 * Num_Runs))
        self.GlobalMeasured_FR = (1.0 * self.TAS_Failure_Number) / (1.0 * Num_Runs)
        self.GlobalMeasured_Cost_TAS = np.sum(self.Vec_fail * self.Vec_prob * self.cumulative_Cost) / (1.0 * Num_Runs)
        self.GlobalMeasured_ResponseTime_TAS = (np.sum(self.Vec_fail * self.Vec_prob * self.cumulative_ResponseTime) / (1.0 * Num_Runs))
        self.GlobalFailureRate = U.T.dot(self.FR_Settings.reshape(self.num_services, 1)).squeeze()
        ######### Monitoring
        self.Monitor_COST_TAS= [ (self.Vec_fail[j] * self.Vec_prob[j] * self.cumulative_Cost[j]) / self.NumCallServices[j] if  self.NumCallServices[j] !=0 else 0.0 for j in range(self.num_Us)]

        self.Monitor_ResponseTime_TAS=[ (self.Vec_fail[j] * self.Vec_prob[j] * self.cumulative_ResponseTime[j]) / self.NumCallServices[j] if  self.NumCallServices[j] !=0 else 0.0 for j in range(self.num_Us)]

        self.Monitor_Failure_TAS=  [ (self.Vec_fail[j] * self.Vec_prob[j] * self.cumulative_FailureRefernces[j]) / self.NumCallServices[j] if  self.NumCallServices[j] !=0 else 0.0 for j in range(self.num_Us)]

        self.ServiceMonitor = [[self.Monitor_COST_TAS[i], self.Monitor_ResponseTime_TAS[i], self.Monitor_Failure_TAS[i]]
                               for i in range(self.num_Us)]

        return np.array([self.GlobalMeasured_Cost_TAS, self.GlobalMeasured_ResponseTime_TAS, self.GlobalFailureReference])



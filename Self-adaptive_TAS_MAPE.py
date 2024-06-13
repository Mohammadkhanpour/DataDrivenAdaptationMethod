import numpy

import MAPELib
from TAS_Simulation_SimCA_MA import *
import cvxpy as cp
from cvxopt import solvers
from cycler import cycler

from MAPELib import *
import numpy as np
from numpy import *
# # Cost , Response Time , Failure Rate(Optimal Goal)
kol=80#40#100#350#120#20
k_change_Setpoint_Cost=20#50#100
k_change_Setpoint_ResponseTime=40#175
k_Change_Setting=1500
k_Failure=60#250
# k_change_Setpoint_EC=50
# k_change_Setpoint_SS=100
# k_Change_Setting=150
# k_Failure=200
k_DeleteGoal=2500
k_planner=20
k_executor=50
k_DeleteGoal_Plot=5 *(k_planner + k_executor)
kol_Plot=6 *(k_planner + k_executor)
DeletedGoal=False
NumDeletedGoal=99
n=3
m=8
MAPELib.m=m
VarMin = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 5 #Lower Bound of Decis                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ion Variables
VarMax = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
MinY=[1.0,1.0,1.0]
MaxY=[1.0,1.0,1.0]
WY= np.array([1.0,1.0,100.0])
WU= np.array([100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0])
ServiceNo_fail=99
Us=[]
Ys=[]
Y_d_plot=[]
Us_Planner=[]
Ys_Planner=[]
Yd_Planner=[]
# Cost , Response Time , Failure Rate(Optimal Goal)
Y_d = np.array([9.0, 30.0, 0.0])
Y_d_plot.insert(len(Y_d_plot), (Y_d.squeeze() * WY).tolist())
W = np.array([30.0, 100.0 ,1.e-10])  #np.array([3.0, 1.0, 1.e-10])#np.array([3.0, 1.0, 1.e-10])  #0.01, 100.0 ,1.e-10000
# u_0=np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1])#np.array([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])#np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1]) #np.array([[0.2,0.3,0.0,0.2,0.3]])
# Us.insert(len(Us),(u_0.squeeze() * WU).tolist())#u_1)
# obj= TAS_Sim_System_MA()
# y_0=obj.TAS_Run(u_0.reshape(m,1)).reshape(n,1)
# yy=y_0.squeeze()
# Ys.insert(len(Ys),(yy * WY).tolist() )
u_1=np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1])#np.array([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])#np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1]) #np.array([[0.2,0.3,0.0,0.2,0.3]])
Us.insert(len(Us),(u_1.squeeze() * WU).tolist())#u_1)
obj= TAS_Sim_System_MA()
y_1=obj.TAS_Run(u_1.reshape(m,1)).reshape(n,1)
yy=y_1.squeeze()
Ys.insert(len(Ys),(yy * WY).tolist() )#[yy[0] * 100.0 ,yy[1] , yy[2]])
ServiceMonitor1=obj.ServiceMonitor
SysType = 'TAS'
Phy=np.ones((n,m))#np.array([[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0]])
################## System Dynamics_Parameters
# diff_u_1 = u_1.reshape(1, m) - u_0.reshape(1, m)  # shape 1,5
# diff_y_1 = y_1.reshape(n, 1) - y_0.reshape(n, 1)  # shape 3,1
# n2 = np.linalg.norm(diff_u_1)
#         ####################
#         ##### System Dynamics
#         ######################
# if n2 == 0.0:
#     Coefficient_Phy = np.zeros(m).reshape(m,1)# np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
# else:
#     Coefficient_Phy = diff_u_1 / n2
#
# Diff_Error = diff_y_1 - Phy.dot(diff_u_1.reshape(m, 1))
#
# Phy_Update = Phy + np.squeeze(Coefficient_Phy * Diff_Error[:, None])
# Phy = Phy_Update

u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner=Planner(k_planner,Phy,W,Y_d,u_1,y_1,VarMin,VarMax,'normal',SysType,TAS_Sim_System_MA)
Ys,Us,Y_d_plot=Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner,Ys,Us,Y_d_plot,WY,WU)
NewPlan=True
ServiceMonitor0=ServiceMonitor1
ServiceChanged=False
ep=0
for k in range(kol):
    #########################
    ####   Executor
    #######################

    Us.insert(len(Us), (u_1.squeeze() * WU).tolist())
    obj = TAS_Sim_System_MA()
    y_1 = obj.TAS_Run(u_1.reshape(m, 1)).reshape(n, 1)
    yy = y_1.squeeze()
    Ys.insert(len(Ys), (yy * WY).tolist() )#[yy[0] * 100.0, yy[1], yy[2]])
    Y_d_plot.insert(len(Y_d_plot), (Y_d.squeeze() * WY).tolist())
    print("Executor - Control Parameter at Iteration {} :{} ".format(k, u_1.squeeze()))
    print("Executor - System Outputs at Iteration {} :{} ".format(k, y_1.squeeze()))
    # if DeletedGoal:
    #     n,Y_d,y_1,Phy,Phy_Update,W=Settings_after_DeleteGoal(n, NumDeletedGoal, Phy, Phy_Update, W, Y_d, y_1)
    if ep==1 :
        NewPlan=False
        ep=0
    if NewPlan :
        ServiceMonitor1= obj.ServiceMonitor
        ep= ep+1

    #########
    #####   Monitor & Analysis
    #################

    if not NewPlan:
        ServiceMonitor0=ServiceMonitor1
        ServiceMonitor1 = obj.ServiceMonitor
        ServiceChanged, ServiceNo_Changed = Monitoring_ChangeSetting(ServiceMonitor0,ServiceMonitor1,MaxY,MinY)
        ServiceNo_fail = Monitoring_ServiceFail(n, ServiceMonitor1, u_1.squeeze())


    ##########################
    ####    Manager
    ##########################
    if ( ServiceNo_fail !=99 or ServiceChanged ):#( Distance_Ys != 0.0):    # Distance_Ys_0 != 0.0 and
        print("Change Detected .....")
        if ServiceChanged and ServiceNo_fail == 99:
            print("System Setting Chanaged :", ServiceNo_Changed )

        # print("Distance Ys:", Distance_Ys)
        if ServiceNo_fail != 99:
            print("ServiceNo_fail :", ServiceNo_fail)
            VarMin[ServiceNo_fail] = 0.0000
            VarMax[ServiceNo_fail] = 0.0000
            TAS_Sim_System_MA.Vec_fail = numpy.ones(m+1)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1, VarMin, VarMax, 'normal',
                                                                    SysType, TAS_Sim_System_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True
        ServiceNo_fail=99
        ServiceChanged = False
    ############################
    #####################
    if k == k_change_Setpoint_Cost:
        print(" Change Setpoint:")

        Y_d = np.array([11.0, 30.0, 0.0])
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1,VarMin,VarMax, 'normal', SysType, TAS_Sim_System_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if k == k_change_Setpoint_ResponseTime:
        print(" Change Setpoint:")
        Y_d = np.array([11.0, 27.0, 0.0])
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1,VarMin,VarMax, 'normal', SysType, TAS_Sim_System_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if (k == k_Change_Setting):
        TAS_Sim_System_MA.EnergyCons_Settings[0]=190# np.array([190, 135, 118, 100, 78])
    if (k==k_Failure) :
        U_Current = u_1.squeeze()
        # Vfail = np.array([1.0,1.0,1.0,0.0,1.0,1.0,1.0,1.0,1.0])#Failling(U_Current)
        TAS_Sim_System_MA.Vec_fail[3] = 0.0 #Vfail
    if k==k_DeleteGoal :
        print("Goal Deleted:")
        DeletedGoal=True
        NumDeletedGoal=0
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner =Planner(k_planner, Phy, W, Y_d, u_1, y_1, VarMin, VarMax, 'normal', SysType, TAS_Sim_System_MA, DeletedGoal,
                NumDeletedGoal)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True

print(len(Ys))
print(Ys)
################
### PLOT
#################
y1=plt.figure("Cost")

#################
plt.plot([i  for i in  list(list(zip(*Ys))[0])]  ,color='k' ,linestyle='solid', label ='Measured')  #i / 10.0
plt.plot(list(zip(*Y_d_plot))[0],color='red', linestyle='dotted', label='Goal')
# plt.plot(list(zip(*Error_Plot))[0],linestyle='solid',color='blue', label='Error')
###################

plt.ylabel('$' ,loc='top' ,rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)
plt.title("Cost" )
plt.grid('on')
y1.savefig('results/Self-Adaptive_TAS/Cost.png')

y2=plt.figure("Response Time")

plt.plot([i  for i in  list(list(zip(*Ys))[1])]  ,color='k' ,linestyle='solid', label ='Measured')  #i / 10.0
plt.plot(list(zip(*Y_d_plot))[1],color='red', linestyle='dotted', label='Goal')
# plt.plot(list(zip(*Error_Plot))[1],linestyle='solid',color='blue', label='Error')
plt.ylabel('ms', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)
plt.title("Response Time")
plt.grid('on')
y2.savefig('results/Self-Adaptive_TAS/Response Time.png')

y3=plt.figure("Failure Rate")
###############################
plt.plot([i  for i in  list(list(zip(*Ys))[2] ) ], color='k' ,linestyle='solid', label='Measured Failure Rate')

plt.ylabel('%', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=1)
plt.title("Failure Rate")
plt.grid('on')
y3.savefig('results/Self-Adaptive_TAS/Failure Rate.png')

Service_MA=plt.figure("Medical Service invocations")
Service_MA.tight_layout()
plt.plot(list(zip(*Us))[0] ,color='red',linestyle='solid', label='S1')
plt.plot(list(zip(*Us))[1] ,color='blue',linestyle= 'dashed', label='S2')
plt.plot(list(zip(*Us))[2] ,color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='S3')
plt.plot(list(zip(*Us))[3] ,color='brown',linestyle= (0,(2,2,10,2,10,2)) , label='S4')
plt.plot(list(zip(*Us))[4] ,color='magenta',linestyle= 'dotted', label='S5')
plt.ylabel('%',loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=5)
plt.title("Medical Analysis (time fractions)" )
plt.grid('on')
Service_MA.savefig('results/Self-Adaptive_TAS/Medical Service invocations.png')

Service_AS=plt.figure("Alarm Service invocations(time fractions)")
Service_AS.tight_layout()
plt.plot(list(zip(*Us))[5] ,color='red',linestyle='solid', label='AS1')
plt.plot(list(zip(*Us))[6] ,color='blue',linestyle= 'dashed', label='AS2')
plt.plot(list(zip(*Us))[7] ,color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='AS3')
plt.ylabel('%',loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=5)
plt.title("Alarm Service (time fractions)" )
plt.grid('on')
Service_AS.savefig('results/Self-Adaptive_TAS/Alarm Service invocations.png')
plt.show()

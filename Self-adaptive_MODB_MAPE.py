import numpy

import MAPELib
import Sw_Simulate_WithSettings_MA
# from Sw_Simulate_WithSettings_MA import *
import cvxpy as cp
from cvxopt import solvers
from cycler import cycler

from MAPELib import *
import numpy as np
from numpy import *
# # Cost , Response Time , Failure Rate(Optimal Goal)
kol=100
k_change_Setpoint_Performance=20#50#100
k_change_Setpoint_Reliability=40#175
k_Change_Setting=60#200#1500
k_Failure=80#200#60#250
k_DeleteGoal=2500
k_planner=100#100#20
k_executor=50
k_DeleteGoal_Plot=5 *(k_planner + k_executor)
kol_Plot=6 *(k_planner + k_executor)
DeletedGoal=False
NumDeletedGoal=99
n=3
m=5
MAPELib.m=m
VarMin = [0.0,0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 5 #Lower Bound of Decis    0.005                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    ion Variables
VarMax = [1.0,1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]  # 0.995
MinY=[1.0,1.0,1.0]
MaxY=[1.0,1.0,1.0]
WY= np.array([1.0,100.0,1.0])
WU= np.array([100.0,100.0,1.0,1.0,1.0])
ServiceNo_fail=99
u_precision=1

Us=[]
Ys=[]
Y_d_plot=[]
Us_Planner=[]
Ys_Planner=[]
Yd_Planner=[]
#Cost, Reliability, Performance
Y_d = np.array([0.1, 0.60, 5.0])
Y_d_plot.insert(len(Y_d_plot), (Y_d.squeeze() * WY).tolist())
W = np.array([1.e-10, 100.0 , 0.01])#np.array([1.e-10, 100.0 , 0.01])  #np.array([3.0, 1.0, 1.e-10])#np.array([3.0, 1.0, 1.e-10])  #0.01, 100.0 ,1.e-10000
# u_0=np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1])#np.array([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])#np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1]) #np.array([[0.2,0.3,0.0,0.2,0.3]])
# Us.insert(len(Us),(u_0.squeeze() * WU).tolist())#u_1)
# obj= TAS_Sim_System_MA()
# y_0=obj.TAS_Run(u_0.reshape(m,1)).reshape(n,1)
# yy=y_0.squeeze()
# Ys.insert(len(Ys),(yy * WY).tolist() )
u_1= np.array([[0.5,0.2,1.0,3.0,2.0]])#np.array([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0])#np.array([0.2,0.1,0.3,0.1,0.3,0.4,0.5,0.1]) #np.array([[0.2,0.3,0.0,0.2,0.3]])
Us.insert(len(Us),(u_1.squeeze() * WU).tolist())#u_1)
obj= SW_Sim_WithSettings_MA()
y_1=obj.SW_Model2(u_1.squeeze()).reshape(n,1)
yy=y_1.squeeze()
Ys.insert(len(Ys),(yy * WY).tolist() )#[yy[0] * 100.0 ,yy[1] , yy[2]])
ServiceMonitor1=obj.ServiceMonitor
SysType = 'SW'
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
u_plan=np.round(u_1,u_precision)
u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner=Planner(k_planner,Phy,W,Y_d,u_plan,y_1,VarMin,VarMax,'normal',SysType,SW_Sim_WithSettings_MA)
Us_Planner_r=[np.round(x,u_precision) for x in Us_Planner]
Ys_Planner_r=[np.array([round(y[0],0),round(y[1],2),round(y[2],1)]) for y in Ys_Planner]
Yd_Planner_r=[np.array([round(y[0],0),round(y[1],2),round(y[2],1)]) for y in Yd_Planner]
Ys,Us,Y_d_plot=Insert_Data_MODB(Us_Planner_r, Ys_Planner_r, Yd_Planner_r,Ys,Us,Y_d_plot,WY,WU)
NewPlan=True
ServiceMonitor0=ServiceMonitor1
ServiceChanged=False
ep=0
for k in range(kol):
    #########################
    ####   Executor
    #######################

    Us.insert(len(Us), (u_1.squeeze() * WU).tolist())
    obj = SW_Sim_WithSettings_MA()
    y_1 = obj.SW_Model2(np.round(u_1.squeeze(),2)).reshape(n, 1)
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
        SrvM= obj.ServiceMonitor
        ServiceMonitor1= [[round(x[0],0),round(x[1],2),round(x[2],0)] for x in SrvM]
        ep= ep+1

    #########
    #####   Monitor & Analysis
    #################

    if not NewPlan:
        ServiceMonitor0=ServiceMonitor1
        SrvM = obj.ServiceMonitor
        ServiceMonitor1=  [[round(x[0],0),round(x[1],2),round(x[2],0)] for x in SrvM]
        # ServiceMonitor1 = obj.ServiceMonitor
        ServiceChanged, ServiceNo_Changed = Monitoring_ChangeSetting(ServiceMonitor0,ServiceMonitor1,MaxY,MinY)
        UM= u_1.squeeze()
        U_Monitor=np.round([UM[0],UM[1],(1.0 - UM[0])*(1.0 - UM[1])])
        ServiceNo_fail = Monitoring_ServiceFail(n, ServiceMonitor1, U_Monitor)


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
            Sw_Simulate_WithSettings_MA.Vec_fail = numpy.ones(m)
        u_plan = np.round(u_1, u_precision)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_plan, y_1, VarMin, VarMax, 'normal',
                                                                    SysType, SW_Sim_WithSettings_MA)

        Us_Planner_r = [np.round(x, u_precision) for x in Us_Planner]
        Ys_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Ys_Planner]
        Yd_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Yd_Planner]
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner_r, Ys_Planner_r, Yd_Planner_r, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True
        ServiceNo_fail=99
        ServiceChanged = False
    ############################
    #####################
    if k == k_change_Setpoint_Performance:
        print(" Change Setpoint:")
        Y_d[2] = 4.0
        u_plan = np.round(u_1, u_precision)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_plan, y_1,VarMin,VarMax, 'normal', SysType, SW_Sim_WithSettings_MA)
        Us_Planner_r = [np.round(x, u_precision) for x in Us_Planner]
        Ys_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Ys_Planner]
        Yd_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Yd_Planner]
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner_r, Ys_Planner_r, Yd_Planner_r, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if k == k_change_Setpoint_Reliability:
        print(" Change Setpoint:")
        Y_d[1] = 0.7
        u_plan = np.round(u_1,u_precision)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_plan, y_1,VarMin,VarMax, 'normal', SysType, SW_Sim_WithSettings_MA)
        Us_Planner_r = [np.round(x, u_precision) for x in Us_Planner]
        Ys_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Ys_Planner]
        Yd_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Yd_Planner]
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner_r, Ys_Planner_r, Yd_Planner_r, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if (k == k_Change_Setting):
        print("MAX:",np.argmax(U_Monitor))
        idx= np.argmax(U_Monitor)
        SetValue= Sw_Simulate_WithSettings_MA.reliability_settings[idx] - 0.1
        print("SetValue: ", SetValue)
        Sw_Simulate_WithSettings_MA.reliability_settings[idx]= SetValue
        # SW_Sim_WithSettings_MA.reliability_settings= np.array([.9, .65, .55]) # np.array([.9, .65, .45])

    if (k==k_Failure) :
        U_Current = U_Monitor
        Vfail = Failling(U_Current)
        SW_Sim_WithSettings_MA.Vec_fail = Vfail
    if k==k_DeleteGoal :
        print("Goal Deleted:")
        DeletedGoal=True
        NumDeletedGoal=0
        u_plan = np.round(u_1, u_precision)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner =Planner(k_planner, Phy, W, Y_d, u_plan, y_1, VarMin, VarMax, 'normal', SysType, SW_Sim_WithSettings_MA, DeletedGoal,
                NumDeletedGoal)
        Us_Planner_r = [np.round(x, u_precision) for x in Us_Planner]
        Ys_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Ys_Planner]
        Yd_Planner_r = [np.array([round(y[0], 0), round(y[1], 2), round(y[2], 1)]) for y in Yd_Planner]
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner_r, Ys_Planner_r, Yd_Planner_r, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True

print(len(Ys))
print(Ys)
################
### PLOT
#################
y1=plt.figure("Cost")
y1.tight_layout()
plt.plot(list(zip(*Ys))[0],color='k', label ='Actual cost')

plt.ylabel('$', loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)
plt.title("Cost" ,fontsize=18)
plt.grid('on')
y1.savefig('results/Self_Adaptive_SW_MODB/Cost.png')

y2=plt.figure("Reliability")
y2.tight_layout()
plt.plot( list(zip(*Ys))[1]  ,color='k', label ='Actual reliability')

plt.plot(list(zip(*Y_d_plot))[1],color='red',linestyle='dashed', label='Reliability goal')

plt.ylabel('%', loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)
plt.title("Reliability" ,fontsize=18)#, x=0.5 ,y=0.9)
plt.grid('on')
y2.savefig('results/Self_Adaptive_SW_MODB/Reliability.png')


y3=plt.figure("Performance")
y3.tight_layout()
plt.plot(list(zip(*Ys))[2],color='k', label='Actual performance')
plt.plot(list(zip(*Y_d_plot))[2],color='red',linestyle='dashed', label='Performance goal')

plt.ylabel('msed', loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)  ## 'upper right'
plt.title("Performance" ,fontsize=18)#, x=0.5 ,y=0.9)
plt.grid('on')
y3.savefig('results/Self_Adaptive_SW_MODB/Performance.png')







Sense=plt.figure("Service Probability")
Sense.tight_layout()
plt.plot(list(zip(*Us))[0],color='red',linestyle='solid', label='Service 1')
plt.plot(list(zip(*Us))[1],color='blue',linestyle= 'dashed', label='Service 2')
plt.plot([(100.0 - x) * (100.0 - y) for x,y in zip(list(zip(*Us))[0],list(zip(*Us))[1])],color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='Service 3')
plt.ylabel('%',loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=5)#
plt.title("Service selection " ,fontsize=18)
plt.grid('on')
Sense.savefig('results/Self_Adaptive_SW_MODB/Service Probability.png')

Service_level=plt.figure("Level of Services")
Service_level.tight_layout()
plt.plot(list(zip(*Us))[2],color='red',linestyle='solid', label='Level of Service 1')
plt.plot(list(zip(*Us))[3],color='blue',linestyle= 'dashed', label='Level of Service 2')
plt.plot(list(zip(*Us))[4],color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='Level of Service 3')
plt.ylabel('%',loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=5)#
plt.title("Level of Services " ,fontsize=18)#
plt.grid('on')
Service_level.savefig('results/Self_Adaptive_SW_MODB/Level of Services.png')

plt.show()



import numpy

from Sw_Simulate_WithSettings import *
import cvxpy as cp
from cvxopt import solvers
from cycler import cycler
from UUV_Simulation_MA import *
from MAPELib import *
import numpy as np
from numpy import *
# Accuracy , Scanning Speed, Energy Consumption
kol=300#120#20
k_change_Setpoint_EC=50
k_change_Setpoint_SS=100
k_Change_Setting=150
k_Failure=200
k_DeleteGoal=250
k_planner=20
k_executor=50
k_DeleteGoal_Plot=5 *(k_planner + k_executor)
kol_Plot=6 *(k_planner + k_executor)
DeletedGoal=False
NumDeletedGoal=99
VarMin = [0.0, 0.0, 0.0, 0.0, 0.0]  # 5 #Lower Bound of Decis                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ion Variables
VarMax = [1.0, 1.0, 1.0, 1.0, 1.0]
MinY=[0.49,2.6,78]
MaxY=[0.97,3.6,170]
WY= np.array([100.0,1.0,1.0])
WU= np.array([100.0,100.0,100.0,100.0,100.0])
n=3
m=5
ServiceNo_fail=99
Us=[]
Ys=[]
Y_d_plot=[]
Us_Planner=[]
Ys_Planner=[]
Yd_Planner=[]
W = np.array([1.e-10, 10.0 , 0.1])#np.array([1.e-10, 100.0 , 0.01])
u_1=np.array([[0.2,0.3,0.1,0.3,0.1]])#np.array([[0.2,0.3,0.0,0.2,0.3]])
Us.insert(len(Us),(u_1.squeeze() * WU).tolist())#u_1)
obj= UUV_Sim_MA()
y_1=obj.UUV_Run(u_1.reshape(m,1)).reshape(n,1)
yy=y_1.squeeze()
Ys.insert(len(Ys),(yy * WY).tolist() )#[yy[0] * 100.0 ,yy[1] , yy[2]])
ServiceMonitor1=obj.ServiceMonitor
Y_d = np.array([1.0, 2.7, 150.0])
Y_d_plot.insert(len(Y_d_plot), (Y_d.squeeze() * WY).tolist())#[Y_d[0], Y_d[1], Y_d[2]])
Phy=np.array([[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0]])
u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner=Planner(k_planner,Phy,W,Y_d,u_1,y_1,VarMin,VarMax,'normal','UUV',UUV_Sim_MA)
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
    obj = UUV_Sim_MA()
    y_1 = obj.UUV_Run(u_1.reshape(m, 1)).reshape(n, 1)
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
            UUV_Sim_MA.Vec_fail = numpy.ones(m)
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1, VarMin, VarMax, 'normal',
                                                                    'UUV', UUV_Sim_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True
        ServiceNo_fail=99
        ServiceChanged = False
    ############################
    #####################
    if k == k_change_Setpoint_EC:
        print(" Change Setpoint:")
        Y_d[2] =142 # np.array([1.0, 2.7, 142.0])
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1,VarMin,VarMax, 'normal', 'UUV', UUV_Sim_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if k == k_change_Setpoint_SS:
        print(" Change Setpoint:")
        Y_d[1] = 3.1# np.array([1.0, 3.1, 142.0])
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner = Planner(k_planner, Phy, W, Y_d, u_1, y_1,VarMin,VarMax, 'normal', 'UUV', UUV_Sim_MA)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan=True
    if (k == k_Change_Setting):
        UUV_Sim_MA.EnergyCons_Settings[0]=190# np.array([190, 135, 118, 100, 78])
    if (k==k_Failure) :
        U_Current = u_1.squeeze()
        Vfail = Failling(U_Current)
        UUV_Sim_MA.Vec_fail = Vfail
    if k==k_DeleteGoal :
        print("Goal Deleted:")
        DeletedGoal=True
        NumDeletedGoal=0
        u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner =Planner(k_planner, Phy, W, Y_d, u_1, y_1, VarMin, VarMax, 'normal', 'UUV', UUV_Sim_MA, DeletedGoal,
                NumDeletedGoal)
        Ys, Us, Y_d_plot = Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner, Ys, Us, Y_d_plot, WY, WU)
        NewPlan = True

print(len(Ys))
print(Ys)
################
### PLOT
#################
y1=plt.figure("Accuracy")


plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_DeleteGoal)]))[0],linestyle='solid', color='blue' , label ='Measured accuracy')
# plt.plot(list(zip(*Ys))[0],linestyle='solid', color='blue' , label ='Measured accuracy')

plt.ylabel('%' ,loc='top' ,rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=1)
plt.title("Accuracy" )
plt.grid('on')
y1.savefig('results/Self_Adaptive_UUV/Accuracy.png')

y2=plt.figure("Scanning Speed")

# plt.plot([i  for i in  list(list(zip(*Ys))[1])]  ,color='k' ,linestyle='solid', label ='Measured speed')
plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_DeleteGoal_Plot)]))[1] ,color='k' ,linestyle='solid', label ='Measured speed')
plt.plot(range(k_DeleteGoal_Plot,kol_Plot-1),list(zip(*[Ys[ki] for ki in range(k_DeleteGoal_Plot,kol_Plot-1)]))[1] ,color='m' ,linestyle='solid', label ='Measured speed')
plt.plot(list(zip(*[Y_d_plot[ki] for ki in range(1,k_DeleteGoal_Plot)]))[1],color='red', linestyle='dotted', label='Speed goal')
# plt.plot(list(zip(*Error_Plot))[1],linestyle='solid',color='blue', label='Speed error')
plt.ylabel('m/s', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)
plt.title("Scanning Speed")
plt.grid('on')
y2.savefig('results/Self_Adaptive_UUV/Scanning Speed.png')

y3=plt.figure("Energy Consumption")
#ax=plt.subplots()
# plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_DeleteGoal-1)]))[2], color='k' ,linestyle='solid', label='Measured energy')
# plt.plot(range(k_DeleteGoal-1,kol-1) , list(zip(*[Ys[ki] for ki in range(k_DeleteGoal-1,kol-1)]))[2],  color='m' ,linestyle='solid', label='Optimal energy' )
# plt.plot(list(zip(*Ys))[2], color='k' ,linestyle='solid', label='Actual energy')
# plt.plot(list(zip(*[Y_d_plot[ki] for ki in range(1,k_DeleteGoal-1)]))[2],color='red', linestyle='dotted', label='Energy goal')
# plt.plot(list(zip(*[Error_Plot[ki] for ki in range(1,k_DeleteGoal-1)]))[2],linestyle='solid' ,color='blue', label='Energy error')
plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_DeleteGoal_Plot)]))[2] ,color='k' ,linestyle='solid', label ='Measured energy')
plt.plot(range(k_DeleteGoal_Plot,kol_Plot-1),list(zip(*[Ys[ki] for ki in range(k_DeleteGoal_Plot,kol_Plot-1)]))[2] ,color='m' ,linestyle='solid', label ='Measured energy')
plt.plot(list(zip(*[Y_d_plot[ki] for ki in range(1,k_DeleteGoal_Plot)]))[2],color='red', linestyle='dotted', label='Energy goal')



plt.ylabel('J/s', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=4)
plt.title("Energy Consumption")
plt.grid('on')
y3.savefig('results/Self_Adaptive_UUV/Energy Consumption.png')

Sense=plt.figure("Sensor Usange")
Sense.tight_layout()
plt.plot(list(zip(*Us))[0],color='red',linestyle='solid', label='Sensor 1')
plt.plot(list(zip(*Us))[1],color='blue',linestyle= 'dashed', label='Sensor 2')
plt.plot(list(zip(*Us))[2],color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='Sensor 3')
plt.plot(list(zip(*Us))[3],color='brown',linestyle= (0,(2,2,10,2,10,2)) , label='Sensor 4')
plt.plot(list(zip(*Us))[4],color='magenta',linestyle= 'dotted', label='Sensor 5')
plt.ylabel('%',loc='top',rotation=0)
plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=5)
plt.title("Sensor usage (time fractions)" )
plt.grid('on')
Sense.savefig('results/Self_Adaptive_UUV/Sensor Usange.png')

plt.show()

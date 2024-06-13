import numpy as np
from Sw_Simulate_WithSettings_MA import *
from TAS_Simulation_SimCA_MA import *
from UUV_Simulation_MA import *
import cvxpy as cp

n=3
m=5

def Planner(MaxIt,Phy,W,Y_d,u_1,y_1,VarMin,VarMax,Mode,SysType,SysClass,DeletedGaol=False,NumDeletedGoal=99):
    global n
    global m

    # SysClass=Sw_Sim
    Ys_Planner = []
    Yd_Planner = []
    Us_Planner = []
    # MaxIt=100
    for k in range(MaxIt):


        ##############################
        #######     Controller
        ##########################
        Y_K = y_1
        U_K = u_1.squeeze()
        if DeletedGaol:
            U_Diff_opt = MOOP(SysType,Mode,U_K,Y_K,Y_d,Phy,W,VarMin,VarMax,DeletedGaol,NumDeletedGoal)
        else:
            U_Diff_opt = MOOP(SysType, Mode, U_K, Y_K, Y_d, Phy, W, VarMin, VarMax)
        U_K_n_opt = U_K.reshape(1, m) + U_Diff_opt.reshape(1, m)  # Shape 1,5

        #############
        #####   Apply to System
        #####################
        u_0 = u_1.reshape(1, m)
        u_1=U_K_n_opt.reshape(1,m) #shape 1,5
        y_0 = y_1.reshape(n, 1)

        obj = SysClass()
        if SysType == 'SW':
            y_1 = obj.SW_Model2(np.round(u_1.squeeze(),1)).reshape(n, 1)
        if SysType == 'UUV':
            y_1 = obj.UUV_Run((u_1).squeeze()).reshape(n, 1)
        if SysType == 'TAS' :
            y_1 = obj.TAS_Run(u_1.reshape(m, 1).squeeze()).reshape(n, 1)
        ####################################################
        ################## System Dynamics_Parameters
        diff_u_1 = u_1.reshape(1, m) - u_0.reshape(1, m)  # shape 1,5
        diff_y_1 = y_1.reshape(n, 1) - y_0.reshape(n, 1)  # shape 3,1
        n2 = np.linalg.norm(diff_u_1)
        ####################
        ##### System Dynamics
        ######################
        if n2 == 0.0:
            Coefficient_Phy = np.zeros(m).reshape(m,1)# np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        else:
            Coefficient_Phy = diff_u_1 / n2

        Diff_Error = diff_y_1 - Phy.dot(diff_u_1.reshape(m, 1))

        Phy_Update = Phy + np.squeeze(Coefficient_Phy * Diff_Error[:, None])
        Phy = Phy_Update


        ########### Show Outputs
        print("Planner- Control Parameter at Iteration {} :{} ".format(k, u_1.squeeze()))
        print("Planner- System Outputs at Iteration {} :{} ".format(k, y_1.squeeze()))
        ##### System Data Gathering
        yy = y_1.squeeze()
        uu = u_1.squeeze()
        Ys_Planner.insert(len(Ys_Planner), yy)  # [yy[0], yy[1] , yy[2]])
        Yd_Planner.insert(len(Yd_Planner), Y_d)  # [Yd[0], Yd[1] , Yd[2]])
        Us_Planner.insert(len(Us_Planner), uu)  # [uu[0], uu[1], uu[2]])

    return u_1, y_1, Phy, Us_Planner, Ys_Planner, Yd_Planner


def MOOP(SysType,Mode,U_K,Y_K,Y_d,Phy,W,VarMin,VarMax,DeletedGaol=False,NumDeletedGoal=99):
    n=3
    global m
    if SysType == 'SW':
        U_Diff_opt = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        X_U_next_p = cp.Variable(2)
        X_U_next_sl = cp.Variable(3, integer=True)
        Diff_Variable = Phy[:, 0:2] @ X_U_next_p + Phy[:, 2:5] @ X_U_next_sl
        # VarMin=[0.005,0.005,1.0,1.0,1.0]
        # VarMax= [0.0995,0.995,5.0,5.0,5.0]
        if Mode == 'normal':
            constraints = [

                           (U_K[0] + X_U_next_p[0]) >= VarMin[0], (U_K[0] + X_U_next_p[0]) <= VarMax[0],
                           (U_K[1] + X_U_next_p[1]) >= VarMin[1], (U_K[1] + X_U_next_p[1]) <= VarMax[1],

                           ###### Check SL Next State
                           (U_K[2] + X_U_next_sl[0]) >= VarMin[2], (U_K[2] + X_U_next_sl[0]) <= VarMax[2],
                           (U_K[3] + X_U_next_sl[1]) >= VarMin[3], (U_K[3] + X_U_next_sl[1]) <= VarMax[3],
                           (U_K[4] + X_U_next_sl[2]) >= VarMin[4], (U_K[4] + X_U_next_sl[2]) <= VarMax[4],

                           ]
        b_Controller = ((Y_d - np.reshape(Y_K, (1, n)))).reshape(n, )
        prob_Controller = cp.Problem(cp.Minimize(cp.norm(W @ cp.square(b_Controller - Diff_Variable), 1)), constraints)
        prob_Controller.solve(cp.settings.MOSEK)
        U_Diff_opt = np.zeros(m).reshape(m,1)#np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
        if prob_Controller.status.find("infeasible") != -1:
            U_Diff_opt = np.zeros(m).reshape(m,1)#np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])

        else:
            U_Diff_opt[0:2] = np.round(X_U_next_p.value,2).reshape(2, 1)
            U_Diff_opt[2:5] = np.round(X_U_next_sl.value,0).reshape(3, 1)


    if SysType == 'UUV':
        X_U_next = cp.Variable(5)
        if Mode == 'normal':
            constraints = [

                (U_K[0] + X_U_next[0]) + (U_K[1] + X_U_next[1]) + (U_K[2] + X_U_next[2]) + (U_K[3] + X_U_next[3]) + (
                        U_K[4] + X_U_next[4]) == 1.0,
                (U_K[0] + X_U_next[0]) >= VarMin[0], (U_K[0] + X_U_next[0]) <= VarMax[0],
                (U_K[1] + X_U_next[1]) >= VarMin[1], (U_K[1] + X_U_next[1]) <= VarMax[1],
                (U_K[2] + X_U_next[2]) >= VarMin[2], (U_K[2] + X_U_next[2]) <= VarMax[2],
                (U_K[3] + X_U_next[3]) >= VarMin[3], (U_K[3] + X_U_next[3]) <= VarMax[3],
                (U_K[4] + X_U_next[4]) >= VarMin[4], (U_K[4] + X_U_next[4]) <= VarMax[4],

            ]
        if DeletedGaol:
            n,Y_d,Y_K,Phy,W=Settings_after_DeleteGoal(n, NumDeletedGoal, Phy, W, Y_d, Y_K)
        Diff_Variable = Phy @ X_U_next
        b_Controller = ((Y_d - np.reshape(Y_K, (1, n)))).reshape(n, )
        prob_Controller = cp.Problem(cp.Minimize(cp.norm(W @ cp.square(b_Controller - Diff_Variable), 1)), constraints)
        prob_Controller.solve(cp.MOSEK)  # (cp.settings.CVXOPT) cp.MOSEK
        # print("status:", prob_Controller.status)
        # print("Cost Value:", prob_Controller.value)
        if prob_Controller.status.find("infeasible") != -1:
            U_Diff_opt = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])

        else:
            U_Diff_opt = (X_U_next.value).reshape(m, 1)  # Shape 5,1

    if SysType == 'TAS':
        X_U_next = cp.Variable(m)
        if Mode == 'normal':
            constraints_TAS = [
                # (U_K + X_U_next) >= VarMin, (U_K + X_U_next) <= VarMax,
                # cp.sum((U_K + X_U_next)[0:5])==1.0,
                # cp.sum((U_K + X_U_next)[5:8]) == 1.0,
                (U_K[0] + X_U_next[0]) >= VarMin[0], (U_K[0] + X_U_next[0]) <= VarMax[0],
                (U_K[1] + X_U_next[1]) >= VarMin[1], (U_K[1] + X_U_next[1]) <= VarMax[1],
                (U_K[2] + X_U_next[2]) >= VarMin[2], (U_K[2] + X_U_next[2]) <= VarMax[2],
                (U_K[3] + X_U_next[3]) >= VarMin[3], (U_K[3] + X_U_next[3]) <= VarMax[3],
                (U_K[4] + X_U_next[4]) >= VarMin[4], (U_K[4] + X_U_next[4]) <= VarMax[4],
                (U_K[5] + X_U_next[5]) >= VarMin[5], (U_K[5] + X_U_next[5]) <= VarMax[5],
                (U_K[6] + X_U_next[6]) >= VarMin[6], (U_K[6] + X_U_next[6]) <= VarMax[6],
                (U_K[7] + X_U_next[7]) >= VarMin[7], (U_K[7] + X_U_next[7]) <= VarMax[7],
                ((U_K[0] + X_U_next[0]) + (U_K[1] + X_U_next[1]) + (U_K[2] + X_U_next[2]) + (U_K[3] + X_U_next[3]) + (
                        U_K[4] + X_U_next[4])) == 1.0,
                ((U_K[5] + X_U_next[5]) + (U_K[6] + X_U_next[6]) + (U_K[7] + X_U_next[7])) == 1.0,

            ]

        Diff_Variable = Phy @ X_U_next
        U_Diff_opt =np.zeros(m).reshape(m,1)#np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        b_Controller = ((Y_d - np.reshape(Y_K, (1, n)))).reshape(n, )
        print("Error:", b_Controller)
        prob_Controller = cp.Problem(cp.Minimize(cp.norm(W @ cp.square(b_Controller - Diff_Variable), 1)), constraints_TAS)

        prob_Controller.solve(cp.settings.MOSEK)
        print("status:", prob_Controller.status)
        print("Cost Value:", prob_Controller.value)
        if prob_Controller.status.find("infeasible") != -1:
            U_Diff_opt = np.zeros(m).reshape(m,1)#np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])

        else:
            U_Diff_opt = (X_U_next.value).reshape(m, 1)

    return   U_Diff_opt #X_U_next, constraints


def Insert_Data_MODB(Us_Planner, Ys_Planner, Yd_Planner,Ys,Us,Y_d_plot,WY,WU):
    for Srvs in Us_Planner:
        Us.insert(len(Us),(Srvs.squeeze() * WU).tolist())


    for yy in Ys_Planner:
        Ys.insert(len(Ys), (yy.squeeze() * WY).tolist())#[yy[0], yy[1] * 100.0, yy[2]])

    for Yd in  Yd_Planner:
        Y_d_plot.insert(len(Y_d_plot), (Yd.squeeze() * WY).tolist())#[Yd[0], Yd[1] * 100.0, Yd[2]])


    return Ys,Us,Y_d_plot

def Monitoring_ServiceFail(n,ServiceMonitor_,U_Monitor):
    ServiceNo_fail=99
    NonZero = U_Monitor[np.round(U_Monitor,3) != 0]  # list(np.nonzero(U_Monitor))
    Pattern = [0.0 for _ in range(n)]
    for i in NonZero:
        index = U_Monitor.tolist().index(i)
        if ServiceMonitor_[index] == Pattern:
            ServiceNo_fail=index

    return ServiceNo_fail

def Ynorm(Y,MaxY,MinY):
    W_norm = np.array([(1.0 / (MaxY[0] - MinY[0])), (1.0 / (MaxY[1] - MinY[1])), (1.0 / (MaxY[2] - MinY[2]))])
    return  np.array(W_norm * (Y.squeeze() - MinY))

def Calc_Distance_norm(y1, y2,MaxY,MinY):
    # Y1_norm = Ynorm(y1, MaxY,MinY)
    # Y2_norm = Ynorm(y2, MaxY,MinY)
    # Distance = np.linalg.norm(Y1_norm - Y2_norm)
    Distance = (np.linalg.norm(y1-y2))

    return Distance

def Round_(y,num):
    return int(y * pow(10,num))

def Monitoring_ChangeSetting(ServiceMonitor1,ServiceMonitor2,MaxY,MinY):
    Distance_Services=[]
    ServiceChanged=False
    S1=np.array(ServiceMonitor1)
    S2=np.array(ServiceMonitor2)

    Distance_Services=[round(Calc_Distance_norm(S1.squeeze()[k], S2.squeeze()[k],MaxY,MinY),1) for k in range(len(S1))]
    # Distance_Services = [Round_(Calc_Distance_norm(S1.squeeze()[k], S2.squeeze()[k], MaxY, MinY), 1) for k in range(len(S1))]
    # print("S1: ",S1)
    # print("S2: ", S2)
    print("Distance_Services: ",Distance_Services)
    ServiceNo_Changed= np.nonzero(Distance_Services)[0]#(Distance_Services[Distance_Services != 0])
    if len(ServiceNo_Changed) != 0 :
        ServiceChanged= True
    if ServiceChanged:
        print("S1: ",S1)
        print("S2: ", S2)


    return ServiceChanged,ServiceNo_Changed
def Failling(u):
    Vfail = np.ones(len(u))
    # non_zero_u = u[u != 0]
    # min_=min(non_zero_u.tolist())
    # index=u.tolist().index(min_)
    Vfail[np.argmin(u[u != 0])] = 0.0
    return Vfail

def Failling2(u):
    Vfail =np.ones(len(u))
    non_zero_u = u[u != 0]
    v = random.choice(non_zero_u)
    index=u.tolist().index(v)
    Vfail[index]=0.0
    return Vfail

def Settings_after_DeleteGoal(n,nd,Phy,W,Y_d,y_1):
    n = n-1
    global m
    Y_d = np.array([3.6, 0.0])# np.delete(Y_d,nd,0)#np.array([3.6, 78.0])
    y_1 = np.delete(y_1, nd, 0)
    # Phy_Update = np.delete(Phy_Update, 0, 0)
    Phy =np.delete(Phy, nd, 0) # np.ones((n,m))#
    W = np.delete(W,nd, 0)
    # ServiceMonitor=np.delete(ServiceMonitor,nd,0)

    return n,Y_d,y_1,Phy,W


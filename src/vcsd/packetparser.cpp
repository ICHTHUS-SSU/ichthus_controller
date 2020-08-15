#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <signal.h>
#include <syslog.h>
#include "umsg.h"

using namespace std;

extern int EthercatManageHandler(char* module, char* arg1, double arg2);
extern int HviManageHandler(char *value);
extern int ObdManageHandler(char *value);
extern int CanManageHandler(char *value);
extern int get_rtParam_int(char *name, char *field);
extern double get_rtParam_double(char *name, char *field);
///added by han 20191010
extern int set_rtParam(char* name, char* field, double val);
extern void sendOdom(double param_val, int bparam);
///
extern int get_cfParam(char *name);
#define MAX_BUFFER 1024
int parseMessage(UserMsg *msg)
{ // NOT COMPLETE!!!
    char arg1[128];
    char arg2[128];
    if (msg->param_id == 0)
    {
	    if (get_cfParam("Main.use_ecat"))
	    {
		    if(EthercatManageHandler("Controller", "HomingPedals", NULL) == 1)
			    return 1;
		    else
		    {
			    cout << "HomingPedals error\n";
			    return -1;
		    }

	    }
	    cout << "check and enable(1) use_ecat in .xml" << endl;
	    //cout << ": homingpedals " << endl;                                     
	    return -1;

    }
    else if (msg->param_id == 1)
    {
	    if (get_cfParam("Main.use_ecat"))
	    {
		    if(EthercatManageHandler("Ecat", "off",NULL) == 1)
			    return 1;
		    else
		    {
			    cout << "EcatOff error\n";
			    return -1;
		    }
	    }
	    cout << "check and enable(1) use_ecat in .xml" << endl;
	    return -1;
    }

   /* else if (regexec_with_args(&regex_get, msg, 2, groups, arg1, NULL))
    {

    if (get_rtParam_double(arg1, "value") != -1)
    {
    cout << "rtParam Matched : " << arg1 << " = " << get_rtParam_double(arg1, "value") << endl;
    goto done;
    }
    if (get_cfParam(arg1) != -1)
    {
    cout << "cfParam Matched : " << arg1 << " = " << get_cfParam(arg1) << endl;
    //cout << ": get " << arg1 << endl;
    goto done;
    }

    goto done;
    }
*/
    else if (msg->param_id == 2)
    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"up");
                break;
            case 1:
                strcpy(arg1,"on");
                break;
            case 2:
                strcpy(arg1,"off");
                break;
            case 3:
                strcpy(arg1,"down");
                break;
        }
            
        if (get_cfParam("Main.use_ecat"))
        {
            syslog(LOG_ERR,"set ecat %s\n",arg1);
            if (EthercatManageHandler("Ecat",arg1,NULL) == 1)
                return 1;
            else
            {
                printf("EM Handler,arg1 error : state name error\n");
                return -1;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : ecat " << arg1 << " " << arg2 << endl;
        return -1;
    }
    else if (msg->param_id == 3)    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"up");
                break;
            case 1:
                strcpy(arg1,"on");
                break;
            case 2:
                strcpy(arg1,"off");
                break;
            case 3:
                strcpy(arg1,"down");
                break;
        }   
        if (get_cfParam("Main.use_obd"))
        {
            syslog(LOG_ERR,"set obd %s\n",arg1);
            if (ObdManageHandler(arg1))
                return 1;
            else
            {
                printf("OM Handler,arg1 error : state name error\n");
                return -1;
            }
        }
        cout << "check and enable(1) use_obd in .xml" << endl;
        //cout << ": set : obd " << arg1 << " " << arg2 << endl;
        return -1;
    }
    else if (msg->param_id == 4)    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"up");
                break;
            case 1:
                strcpy(arg1,"on");
                break;
            case 2:
                strcpy(arg1,"off");
                break;
            case 3:
                strcpy(arg1,"down");
                break;
        }
        if (get_cfParam("Main.use_can"))
        {

            syslog(LOG_ERR,"set can %s\n",arg1);
            if (CanManageHandler(arg1)) {
                return 1;
            }


            else
            {
                printf("CM Handler, arg1 error : state name error\n");
                return -1;
            }

        }
        cout << "check and enable(1) use_can in .xml" << endl;
        //cout << ": set : can " << arg1 << " " << arg2 << endl;
        return -1;
    }
    else if (msg->param_id == 5)    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"up");
                break;
            case 1:
                strcpy(arg1,"on");
                break;
            case 2:
                strcpy(arg1,"off");
                break;
            case 3:
                strcpy(arg1,"down");
                break;
        }
        if (get_cfParam("Main.use_hvi"))
        {
            syslog(LOG_ERR,"set hvi %s\n",arg1);
            if(HviManageHandler(arg1) == 1)
                return 1;
            else
                return -1;

            return 1;
        }
        cout << "check and enable(1) use_hvi in .xml" << endl;
        //cout << ": set : hvi " << arg1 << " " << arg2 << endl;
        return -1;
    }

    else if (msg->param_id == 6)    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"Pullover");
                break;
            case 1:
                strcpy(arg1,"Homingpedals");
                break;
            case 2:
                strcpy(arg1,"Estop");
                break;
            case 3:
                strcpy(arg1,"Selftest");
                break;
	        case 4:
    		    strcpy(arg1,"fixsteer");
		        break;
            case 5:
    		    strcpy(arg1,"ready2start");
		        break;
        }
        if (get_cfParam("Main.use_ecat"))
        {
            syslog(LOG_ERR,"set motion %s\n",arg1);
            if (EthercatManageHandler("Motion", arg1, atof(arg2)))
                return 1;
            else
            {
                printf("EC Handler, arg1 error : Motion name error\n");
                return -1;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : controller " << arg1 << " " << arg2 << endl;
        return -1;
    }
    else if (msg->param_id == 7)    {
        switch((int)msg->param_val)
        {
            case 0:
                strcpy(arg1,"TuneCruiseControl");
                break;
            case 1:
                strcpy(arg1,"SelfDriving");
                break;
            case 2:
                strcpy(arg1,"AVC");
                break;
        }      
        if (get_cfParam("Main.use_ecat"))
        {
            syslog(LOG_ERR,"set contorller %s\n",arg1);
            if (EthercatManageHandler("Controller", arg1, atof(arg2)))
                return 1;
            else
            {
                printf("EC Handler, arg1 error : Controller name error\n");
                return -1;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : controller " << arg1 << " " << arg2 << endl;
        return -1;
    }
    //added by han 20191010
    else if(msg->param_id==30){
        if (get_cfParam("Main.use_ecat")){
            float ndt_cur_vel;
            ndt_cur_vel = msg->param_val;
            if (ndt_cur_vel < 80){
                set_rtParam("OBD.actual_velocity", "value", ndt_cur_vel);
                if(get_rtParam_int("OBD.publish_to_agent","value") == 1 ){
                    sendOdom(ndt_cur_vel/3.6,2);
                }
                return 1;
            }
            else cout<<"ndt_cur_vel : "<<ndt_cur_vel<<"is higher than maximum check ndt"<<endl;
        }
        else cout << "check and enable(1) use_ecat in .xml" << endl;
    }
    ///
    else{
         switch((int)msg->param_id)
        {
            case 8:
                strcpy(arg1,"t.tpos");
                break;
            case 9:
                strcpy(arg1,"b.tpos");
                break;
            case 10:
                strcpy(arg1,"l.ctpos");
                break;
            case 11:
                strcpy(arg1,"l.ltpos");
                break;
            case 12:
                strcpy(arg1,"l.rtpos");
                break;
            case 13:
                strcpy(arg1,"pl.mode");
                break;
            case 14:
                strcpy(arg1,"s.tpos");
                break;
            case 15:
                strcpy(arg1,"g.tpos");
                break;
            case 16:
                strcpy(arg1,"cc.tvelo");
                break;
            case 17:
                strcpy(arg1,"sc.tanvelo");
                break;
            case 18:
                strcpy(arg1,"hvi.mode");
                break;
            case 19:
                strcpy(arg1,"CAN.publish_to_agent");
                break;
            case 20:
                strcpy(arg1,"ECAT.num_motors");
                break;
        }      
        //sprintf(arg2, "%lf", msg->param_val);
        if (get_cfParam("Main.use_ecat"))
        {
            printf("arg1 : %s\n",arg1);
            printf("param->val : %lf\n",msg->param_val);
            if(get_rtParam_int("Motion.state", "value") != 10) 
                syslog(LOG_ERR,"set param %s : %lf\n",arg1,msg->param_val);
            if (EthercatManageHandler("Param", arg1, msg->param_val) == 1)
                return 1;
            else
            {
                printf("EC Handler, arg1 error : Param name error\n");
                return -1;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : param " << arg1 << " " << arg2 << endl;
        return -1;

    }
    printf("Unknown command\n", msg);
    return -1;
}
}

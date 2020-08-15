#include <regex.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <signal.h>

using namespace std;

extern int EthercatManageHandler(char* module, char* arg1, double arg2);
extern int ObdManageHandler(char *value);
extern int CanManageHandler(char *value);
extern int HviManageHandler(char *value);
extern double get_rtParam_double(char *name, char *field);
extern int get_cfParam(char *name);
#define MAX_BUFFER 1024
// regex with one arg
regex_t regex_help;
regex_t regex_quit;
regex_t regex_show;
regex_t regex_homingpedals;
regex_t regex_ecatoff;
regex_t regex_estop;

// regex with two args
regex_t regex_get;

// regex with three args
regex_t regex_set; //tvel tangle clidar llidar rlidar gearstick
                   //steer throttle brake signtower buzzer
regex_t regex_set_ecat;
regex_t regex_set_obd;
regex_t regex_set_can;
regex_t regex_set_hvi;

regex_t regex_set_param;
regex_t regex_set_motion;
regex_t regex_set_controller;

void regcomp_all()
{ // NOT COMPLETE!!!
    regcomp(&regex_help, "^help$", REG_EXTENDED);
    regcomp(&regex_quit, "^quit$", REG_EXTENDED);
    regcomp(&regex_show, "^show$", REG_EXTENDED);

    regcomp(&regex_homingpedals, "^h$", REG_EXTENDED);
    regcomp(&regex_ecatoff, "^f$", REG_EXTENDED);
    regcomp(&regex_estop, "^e$", REG_EXTENDED);

    regcomp(&regex_get, "^get ([a-zA-Z0-9_.]{1,40})$", REG_EXTENDED);
    

    regcomp(&regex_set_ecat, "^set ecat ([a-zA-Z0-9_]{1,5})$", REG_EXTENDED);
    regcomp(&regex_set_obd, "^set obd ([a-zA-Z0-9_]{1,5})$", REG_EXTENDED);
    regcomp(&regex_set_can, "^set can ([a-zA-Z0-9_]{1,5})$", REG_EXTENDED);
    regcomp(&regex_set_hvi, "^set hvi ([a-zA-Z0-9_]{1,5})$", REG_EXTENDED);

    //regcomp(&regex_set_param, "^set param ([a-zA-Z0-9_.]{1,40}) ([a-zA-Z0-9_.-]{0,19})$", REG_EXTENDED);
    regcomp(&regex_set_motion, "^set motion ([a-zA-Z0-9_.]{1,19})$", REG_EXTENDED);
    regcomp(&regex_set_controller, "^set controller ([a-zA-Z0-9_.]{1,19})$", REG_EXTENDED);
    regcomp(&regex_set, "^set ([a-zA-Z0-9_.]{1,40}) ([a-zA-Z0-9_.-]{1,19})$", REG_EXTENDED);
}

void regfree_all()
{ // NOT COMPLETE!!!
    regfree(&regex_help);
    regfree(&regex_quit);
    regfree(&regex_show);

    regfree(&regex_homingpedals);
    regfree(&regex_ecatoff);
    regfree(&regex_estop);

    regfree(&regex_get);
    
    regfree(&regex_set_ecat);
    regfree(&regex_set_obd);
    regfree(&regex_set_can);
    regfree(&regex_set_hvi);

    //regfree(&regex_set_param);
    regfree(&regex_set);
    regfree(&regex_set_motion);
    regfree(&regex_set_controller);
}

int regexec_with_args(regex_t *regex, char *msg, int ngroups, regmatch_t *groups,
                      char *arg1, char *arg2)
{
    int ret = regexec(regex, msg, ngroups, groups, 0);
    if (ret == 0)
    {
        int len;
        if (ngroups > 1)
        {
            len = groups[1].rm_eo - groups[1].rm_so;
            memcpy(arg1, msg + groups[1].rm_so, len);
            arg1[len] = '\0';
        }
        if (ngroups > 2)
        {
            len = groups[2].rm_eo - groups[2].rm_so;
            memcpy(arg2, msg + groups[2].rm_so, len);
            arg2[len] = '\0';
        }
    }
    return !ret;
}

void chomp(char *msg, int len)
{
    int nconsumed, nread = len;
    char *npos, *rpos, *cpos = msg;
    while ((npos = (char *)memchr(cpos, '\n', nread)) != NULL)
    {
        *npos = 0; // replace '\n' with NULL
        if ((rpos = (char *)memchr(cpos, '\r', nread)) != NULL)
            *rpos = 0; // replace '\r' with NULL
        nconsumed = npos - cpos + 1;
        nread -= nconsumed;
        cpos = npos + 1;
    }
}

void _reg_process(char *msg)
{ // NOT COMPLETE!!!
    regmatch_t groups[3];
    char arg1[128];
    char arg2[128];

    chomp(msg, strlen(msg)); // remove \r\n from message
    //regcomp_all();

    if (regexec_with_args(&regex_help, msg, 0, NULL, NULL, NULL))
    {
        //do_help();
        //cout << ": help" << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_quit, msg, 0, NULL, NULL, NULL))
    {
        //do_quit();
        //cout << ": quit" << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_show, msg, 0, NULL, NULL, NULL))
    {
        //do_show();
        //cout << ": show " << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_homingpedals, msg, 0, NULL, NULL, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if(EthercatManageHandler("Controller", "HomingPedals", NULL) == 1)
                goto done;
            else
            {
                cout << "HomingPedals error\n";
                goto done;
            }
            
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": homingpedals " << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_ecatoff, msg, 0, NULL, NULL, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if(EthercatManageHandler("Ecat", "off",NULL) == 1)
                goto done;
            else
            {
                cout << "EcatOff error\n";
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        goto done;
    }
    
    else if (regexec_with_args(&regex_estop, msg, 0, NULL, NULL, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if(EthercatManageHandler("Controller", "Estop", NULL) == 1)
                goto done;
            else
            {
                cout << "Estop error\n";
            }
            
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_get, msg, 2, groups, arg1, NULL))
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
    
    else if (regexec_with_args(&regex_set_ecat, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if (EthercatManageHandler("Ecat",arg1,NULL) == 1)
                goto done;
            else
            {
                printf("EtherCAT Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : ecat " << arg1 << " " << arg2 << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_set_obd, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_obd"))
        {
            if (ObdManageHandler(arg1) == 1)
                goto done;
            else
            {
                printf("OBD Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_obd in .xml" << endl;
        //cout << ": set : obd " << arg1 << " " << arg2 << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_set_can, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_can"))
        {
            
            if (CanManageHandler(arg1) == 1)
            {
                goto done;
            }
            

            else
            {
                printf("CAN Manage Handler error\n");
                goto done;
            }
            
        }
        cout << "check and enable(1) use_can in .xml" << endl;
        //cout << ": set : can " << arg1 << " " << arg2 << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_set_hvi, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_hvi"))
        {
            if (HviManageHandler(arg1) == 1)
            {
                goto done;
            }
            

            else
            {
                printf("HVI Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_hvi in .xml" << endl;
        //cout << ": set : hvi " << arg1 << " " << arg2 << endl;
        goto done;
    }
    
    else if (regexec_with_args(&regex_set_motion, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if (EthercatManageHandler("Motion", arg1, atof(arg2)) == 1)
                goto done;
            else
            {
                printf("Motion Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : controller " << arg1 << " " << arg2 << endl;
        goto done;
    }
    else if (regexec_with_args(&regex_set_controller, msg, 2, groups, arg1, NULL))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if (EthercatManageHandler("Controller", arg1, atof(arg2)) == 1)
                goto done;
            else
            {
                printf("Controller Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : controller " << arg1 << " " << arg2 << endl;
        goto done;
    }
    /*else if (regexec_with_args(&regex_set_param, msg, 3, groups, arg1, arg2))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if (EthercatManageHandler("Param", arg1, atof(arg2)))
                goto done;
            else
            {
                printf("EC Handler, arg1 error : Param name error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : param " << arg1 << " " << arg2 << endl;
        goto done;
    }*/
    else if (regexec_with_args(&regex_set, msg, 3, groups, arg1, arg2))
    {
        if (get_cfParam("Main.use_ecat"))
        {
            if (EthercatManageHandler("Param", arg1, atof(arg2)) == 1)
                goto done;
            else
            {
                printf("Param Manage Handler error\n");
                goto done;
            }
        }
        cout << "check and enable(1) use_ecat in .xml" << endl;
        //cout << ": set : param " << arg1 << " " << arg2 << endl;
        goto done;
    }
    printf("Unknown command\n", msg);

done:
    //regfree_all();
    return;
}

int reg_process(char *msg)
{
    _reg_process(msg);
    return 0;
}

void print_string(char *p)
{
    while (*p != 0)
        printf("%u_", (unsigned char)*p++);
    printf("\n");
}

void read_thread_handler(void *arg)
{
    char buf[MAX_BUFFER], buffer[MAX_BUFFER];
    //int sockfd = *((int *)arg);
    int nread, nconsumed, totread = 0;
    char *cpos, *npos, *rpos;

    //pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    //pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    memset(buffer, 0, sizeof(buffer));

    regcomp_all();

    while (1)
    {
        memset(buf, 0, MAX_BUFFER);
        nread = read(STDIN_FILENO, buf, sizeof(buf));
        if (nread < 0)
        {

            kill(getpid(), SIGINT);
        }
        /*else if (nread == 0) {
	      printf("recv_thread: socket closed\n");
	      kill(getpid(), SIGINT);
	    }*/
        // else if (nread > 0)

        if (sizeof(buffer) - strlen(buffer) <= strlen(buf))
        {
            printf("read_thread: too small buffer\n");
            kill(getpid(), SIGINT);
        }

        strcat(buffer, buf);
        totread += nread;
        cpos = buffer;
        while ((npos = (char *)memchr(cpos, '\n', totread)) != NULL)
        {
            *npos = 0; // replace '\n' with NULL
            if ((rpos = (char *)memchr(cpos, '\r', totread)) != NULL)
                *rpos = 0; // replace '\r' with NULL
            //printf("read_thread: %s\n", cpos);
            if (reg_process(cpos) != 0)
                kill(getpid(), SIGINT);
            nconsumed = npos - cpos + 1;
            totread -= nconsumed;
            cpos = npos + 1;
        }
        //memset(buf, 0, MAX_BUFFER);
        strcpy(buf, cpos);
        memset(buffer, 0, MAX_BUFFER);
        strcpy(buffer, buf);
        if (strlen(buffer) > 0)
        {
            printf("read_thread: %s (in buffer)\n", buffer);
            print_string(buffer);
        }
    }

    regfree_all();
}
/*int main(int argc, char *argv[])
{

	 regcomp_all();// compile all regex only once

	while (1)
	{

		read_thread_handler();
	}

	regfree_all(); // free all regex

	return 0;
}*/

#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
//#include "gpionum.h" /////gpio
#include <stdio.h>
#include <iostream>

#include <syslog.h> //added by hyo
#include <fcntl.h>  //added by hyo

#include "vthread.h"
#include "socketprogram.h"
#include "hybridautomata.h"

#define LOCKFILE "/var/run/vcsd.pid"
#define LOCKMODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)

using namespace std;

extern int readVehicleConfigs(char *xml_file); // from xml.cpp
extern int get_cfParam(char *name);            // from GlobalParam.cpp
extern int set_rtParam(char *name, char *field, double val);
extern int get_rtParam_int(char *name, char *field);
extern double get_rtParam_double(char *name, char *field);

extern void initEcatModule(); //from EthercatManager.cpp
extern void initObdModule();
extern void initCanModule();
extern void initHviModule();
//extern int EthercatManageHandler(int value);
extern int EthercatCyclicHandler(char *arg1, double arg2);

extern void read_thread_handler(void *arg);
extern int ecatOff();
extern int ecatDown();
extern int obdOff();
extern int obdDown();
extern int hviDown();
extern int parseMessage(UserMsg *msg); //added by hyo

pthread_t recv_thread[MAX_CLIENTS];
pthread_t read_thread;
VThread *send_thread[MAX_CLIENTS] = {NULL};
//VThread *EM_thread;

bool is_first_connection = true;
int num_of_clients = 0;
int client_idx;
void printCar()
{
    printf("                                                                   ___\n");
    printf("                                                                  {___}\n");
    printf("                                                         __   _____//_____   __\n");
    printf("                                                   ___  (0 ) /            \\ ( 0)  ___ \n");
    printf("                                                  [___] _[]_/              \\_[]_ [___]\n");
    printf("                                                    \\\\ /         [   ]          \\ //\n");
    printf("                                                     \\/                          \\/\n");
    printf("                                               __..--{____________________________}-..___           \n");
    printf("                                           _.-\"\"\"\"\"\"\"\"\"\"\"-----....      ____________     `.             \n");
    printf("                                        .-\"                       l ,-\"\"     \\      \"-.    `.          \n");
    printf("                                     .-\"                          ; ;         ;        \\    \"   \n");
    printf("                                  .'                             : :          |        ;    .l  \n");
    printf("                            _.._.'                              ; ;  ___     |        ;   .':  \n");
    printf("                           (  .'                               : :  :   \".   : ...-'-/   .'  ; \n");
    printf("                            )'                                | ;  ; __.'-\"(     .'  .--.:   ;\n");
    printf("                    ___...-'\"\"\"\"----....__..................-' :-/.'       \\_.-'  .' .-.\\l   ;\n");
    printf("            __..--\"\"__   ___  ___    __  ___ __  _____  __      /\\\"          ;    / .gs./\\; ;\n");
    printf("        _.-\"       / _\\ /___\\/___\\/\\ \\ \\/ _ / _\\ \\_   \\/ /     /  ;          |   . d$P\"Tb   ;\n");
    printf("     .-\"           \\ \\ //  ///  //  \\/ / /_\\\\ \\   / /\\/ /     /   |          :   ;:$$   $;  ;\n");
    printf("   .'             __\\ / \\_// \\_// /\\  / /_\\\\_\\ \\_/ /_/ /___  /    |          |   $$$;   :$ -\n");
    printf("  /\"-._           \\__/\\___/\\___/\\_\\ \\/\\____/\\__\\____/\\____/ /     :          ;  _$$$;   :$ \n");
    printf(" :     \"\"-_..______________.... _ ,_,_ ,__--..__ . .-\"\"\"-. /      |         _:-\" $$$;   :$ \n");
    printf(" ;__     ;                     ;              -'/  .d$$$b.        ;      .-\".'   :$$$   $P \n");
    printf(";   -\"\" /   .----...____        \"-+_______--\"\"\":  dP' `T$P        |   .-\" .' _.gd$$$$b_d$' \n");
    printf(";    __...---|  ICHTHUS  |----....____         | :$     $b        : .'   (.-\"  `T$$$$$$P'  \n");
    printf(";  .';       '----...____;            \"-.      ; $;     :$;_____..-\"  .-\"                  \n");
    printf(": /  :                                   \\__..-':$       $$ ;-.    .-\"                     \n");
    printf(" Y    ;       /\"\"\"\"----...:________.....__;_gg$$$$;      :$;|  `.-\"                        \n");
    printf(" :    :$$$$$$$                              gg$$$$       $$;:.-\"                           \n");
    printf("  $$$$$$$$$$$$                               g$$$$;     :$$                                \n");
    printf("  'T$$$$$$$$P'                               T$$$$$b._.d$P                               \n");
    printf("    `T$$$$P'                                  T$$$$$$$$$P                              \n\n");

    syslog(LOG_ERR, "\n                                                               ___\n");
    syslog(LOG_ERR, "                                                                  {___}\n");
    syslog(LOG_ERR, "                                                         __   _____//_____   __\n");
    syslog(LOG_ERR, "                                                   ___  (0 ) /            \\ ( 0)  ___ \n");
    syslog(LOG_ERR, "                                                  [___] _[]_/              \\_[]_ [___]\n");
    syslog(LOG_ERR, "                                                    \\\\ /         [   ]          \\ //\n");
    syslog(LOG_ERR, "                                                     \\/                          \\/\n");
    syslog(LOG_ERR, "                                               __..--{____________________________}-..___           \n");
    syslog(LOG_ERR, "                                           _.-\"\"\"\"\"\"\"\"\"\"\"-----....      ____________     `.             \n");
    syslog(LOG_ERR, "                                        .-\"                       l ,-\"\"     \\      \"-.    `.          \n");
    syslog(LOG_ERR, "                                     .-\"                          ; ;         ;        \\    \"   \n");
    syslog(LOG_ERR, "                                  .'                             : :          |        ;    .l  \n");
    syslog(LOG_ERR, "                            _.._.'                              ; ;  ___     |        ;   .':  \n");
    syslog(LOG_ERR, "                           (  .'                               : :  :   \".   : ...-'-/   .'  ; \n");
    syslog(LOG_ERR, "                            )'                                | ;  ; __.'-\"(     .'  .--.:   ;\n");
    syslog(LOG_ERR, "                    ___...-'\"\"\"\"----....__..................-' :-/.'       \\_.-'  .' .-.\\l   ;\n");
    syslog(LOG_ERR, "            __..--\"\"__   ___  ___    __  ___ __  _____  __      /\\\"          ;    / .gs./\\; ;\n");
    syslog(LOG_ERR, "        _.-\"       / _\\ /___\\/___\\/\\ \\ \\/ _ / _\\ \\_   \\/ /     /  ;          |   . d$P\"Tb   ;\n");
    syslog(LOG_ERR, "     .-\"           \\ \\ //  ///  //  \\/ / /_\\\\ \\   / /\\/ /     /   |          :   ;:$$   $;  ;\n");
    syslog(LOG_ERR, "   .'             __\\ / \\_// \\_// /\\  / /_\\\\_\\ \\_/ /_/ /___  /    |          |   $$$;   :$ -\n");
    syslog(LOG_ERR, "  /\"-._           \\__/\\___/\\___/\\_\\ \\/\\____/\\__\\____/\\____/ /     :          ;  _$$$;   :$ \n");
    syslog(LOG_ERR, " :     \"\"-_..______________.... _ ,_,_ ,__--..__ . .-\"\"\"-. /      |         _:-\" $$$;   :$ \n");
    syslog(LOG_ERR, " ;__     ;                     ;              -'/  .d$$$b.        ;      .-\".'   :$$$   $P \n");
    syslog(LOG_ERR, ";   -\"\" /    .----...____       \"-+_______--\"\"\":  dP' `T$P        |   .-\" .' _.gd$$$$b_d$' \n");
    syslog(LOG_ERR, ";    __...---|  ICHTHUS  |----....____         | :$     $b        : .'   (.-\"  `T$$$$$$P'  \n");
    syslog(LOG_ERR, ";  .';       '----...____;            \"-.      ; $;     :$;_____..-\"  .-\"                  \n");
    syslog(LOG_ERR, ": /  :                                   \\__..-':$       $$ ;-.    .-\"                     \n");
    syslog(LOG_ERR, " Y    ;       /\"\"\"\"----...:________.....__;_gg$$$$;      :$;|  `.-\"                        \n");
    syslog(LOG_ERR, " :    :$$$$$$$                              gg$$$$       $$;:.-\"                           \n");
    syslog(LOG_ERR, "  $$$$$$$$$$$$                               g$$$$;     :$$                                \n");
    syslog(LOG_ERR, "  'T$$$$$$$$P'                               T$$$$$b._.d$P                               \n");
    syslog(LOG_ERR, "    `T$$$$P'                                  T$$$$$$$$$P                              \n\n");
}
void printLogo()
{
    printf("\n                   _ _\n");
    printf("              ,-;\"-;\"=-\",-:.     -=;\n");
    printf("            ,'=/          `=`, '-=/   \n");
    printf("           ==/              -==/    \n");
    printf("            -<=`.         ,=,-<=`.  \n");
    printf("              `-=_,=_,_-,'-'    -=;\n\n");
    printf("\n _____ _____ _    _ _______ _    _ _    _  _____ \n");
    printf("|_   _/ ____| |  | |__   __| |  | | |  | |/ ____|\n");
    printf("  | || |    | |__| |  | |  | |__| | |  | | (___  \n");
    printf("  | || |    |  __  |  | |  |  __  | |  | |\\___ \\ \n");
    printf("  | || |____| |  | |  | |  | |  | | |__| |____) |\n");
    printf("|_____\\_____|_|  |_|  |_|  |_|  |_|\\____/|_____/ \n\n");

    syslog(LOG_ERR, "                   _ _\n");
    syslog(LOG_ERR, "              ,-;\"-;\"=-\",:-.     -=;\n");
    syslog(LOG_ERR, "            ,'=/          `=`, '-=/   \n");
    syslog(LOG_ERR, "           ==/              -==/    \n");
    syslog(LOG_ERR, "            -<=`.         ,=,-<=`.  \n");
    syslog(LOG_ERR, "              `-=_,=_,_-,'-'    -=;\n");
    syslog(LOG_ERR, " _____ _____ _    _ _______ _    _ _    _  _____ \n");
    syslog(LOG_ERR, "|_   _/ ____| |  | |__   __| |  | | |  | |/ ____|\n");
    syslog(LOG_ERR, "  | || |    | |__| |  | |  | |__| | |  | | (___  \n");
    syslog(LOG_ERR, "  | || |    |  __  |  | |  |  __  | |  | |\\___ \\ \n");
    syslog(LOG_ERR, "  | || |____| |  | |  | |  | |  | | |__| |____) |\n");
    syslog(LOG_ERR, "|_____\\_____|_|  |_|  |_|  |_|  |_|\\____/|_____/ \n");
}
void exitHandler(VThread *t)
{
    cout << t->get_thread_name() << " terminated" << endl;
}
void myFlush()
{
    while (getchar() != '\n')
        ;
}

void countDown(int sec)
{
    for (int i = 0; i < sec; i++)
    {
        sleep(1);
        cout << "countDown to shutDown: " << sec - i << endl;
    }
}

void shutDown()
{
    cout << "shutDown Function" << endl;

    void *retval = 0;
    int idx = 0;

    is_first_connection = true;

    EthercatCyclicHandler("HomingPedals", NULL);
    sleep(1);
    ecatOff();
    sleep(1);
    pthread_cancel(read_thread);
    pthread_join(read_thread, &retval);
    if ((int)retval == PTHREAD_CANCELED)
        cout << "read_thread canceled" << endl;
    else
        cout << "read_thread cancellation failed" << endl;

    while (idx < num_of_clients)
    {
        if (recv_thread[idx] != 0)
        {
            pthread_cancel(recv_thread[idx]);
            pthread_join(recv_thread[idx], &retval);
            if ((int)retval == PTHREAD_CANCELED)
                cout << "recv_thread canceled" << endl;
            else
                cout << "recv_thread cancellation failed" << endl;
            recv_thread[idx] = 0;
        }
        ++idx;
    }
    idx = 0;

    while (idx < num_of_clients)
    {
        //delete send_thread[idx]; send_thread[idx]=NULL;
        idx++;
    }

    //delete parsing_thread;
    obdOff();
    //countDown(3);
    obdDown();
    ecatDown();
    //myFlush();
    exit(0);
}
void signalHandler(int signo)
{
    if ((signo = !SIGINT) || (signo = !SIGTERM))
    {
        cout << "unexpected signal = " << signo << " '" << strerror(signo) << "'" << endl;
        exit(0);
    }
    shutDown();
    exit(0);
}
int lockfile(int fd) //added by hyo//
{
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_start = 0;
    fl.l_whence = SEEK_SET;
    fl.l_len = 0;
    return (fcntl(fd, F_SETLK, &fl));
}

int already_running(void) //added by hyo//
{
    int fd;
    char buf[16];

    fd = open(LOCKFILE, O_RDWR | O_CREAT, LOCKMODE);
    if (fd < 0)
    {
        syslog(LOG_ERR, "can't open %s: %s", LOCKFILE, strerror(errno));
        exit(1);
    }
    if (lockfile(fd) < 0)
    {
        if (errno == EACCES || errno == EAGAIN)
        {
            close(fd);
            return (1);
        }
        syslog(LOG_ERR, "can't lock %s: %s", LOCKFILE, strerror(errno));
        exit(1);
    }
    ftruncate(fd, 0);
    sprintf(buf, "%ld", (long)getpid());
    write(fd, buf, strlen(buf) + 1);
    return (0);
}

void sendAck(UserMsg *p, int ret_code) //send ack or nack msg
{
    if (get_cfParam("Main.use_socket"))
    {
        UserMsg *packet = new UserMsg;
        int seq_no = p->seq_no + 1;
        int ack_no = p->seq_no;
        *packet = UserMsg(seq_no, ack_no, p->cmd_code, p->param_id, p->param_val, ret_code, p->result_msg);

        send_thread[client_idx]->PostMsg((const UserMsg *)packet);
    }
}
void sendDebug(char *ret_msg) //send debug msg
{
    static unsigned int seq_no = 0;
    if (get_cfParam("Main.use_socket"))
    {
        UserMsg *packet = new UserMsg;
        *packet = UserMsg(++seq_no, 0, 2, 0, 0, 0, ret_msg); //debug msg has ack_no 0
                                                             //syslog(LOG_ERR,"%s",ret_msg);

        send_thread[client_idx]->PostMsg((const UserMsg *)packet);
    }
}
void sendOdom(double param_val, int bparam)//bparam : { actual v = 0 } { steer ang = 1 }
{
    UserMsg *packet = new UserMsg;
    int cmd_code = 0;
    int param_id;
    static unsigned int seq_no =0;
    char ret_msg[20];

    if(bparam == 0)
    {
        param_id = 20;//param_id : { actual vel = 20 }
    	strcpy(ret_msg, "CAN.actual_velocity");
    }
    else if (bparam == 1)
    {
        param_id = 21;//param_id : { steer ang = 21 }
        strcpy(ret_msg, "CAN.steering_angle");
    }
    else if(bparam == 2)
    {
        param_id = 22;//param_id : { actual vel = 20 }
    	strcpy(ret_msg, "OBD.actual_velocity");
    }
    else if (bparam == 3)
    {
        param_id = 23;//param_id : { steer ang = 21 }
        strcpy(ret_msg, "ECAT.steering_angle");
    }
    else if(bparam == 4)
    {
        param_id = 24;//param_id : { actual vel = 20 }
    	strcpy(ret_msg, "CAN.Gway_Wheel_Velocity_RL");
    }
    else if (bparam == 5)
    {
        param_id = 25;//param_id : { steer ang = 21 }
        strcpy(ret_msg, "CAN.Gway_Wheel_Velocity_RR");
    }
    else return;

    *packet = UserMsg(++seq_no, 0, cmd_code, param_id, param_val, 0, ret_msg);

    send_thread[client_idx]->PostMsg((const UserMsg *)packet);
}

string current_working_directory()
{
    char* cwd = getcwd(0,0);
    string working_directory(cwd);
    free(cwd);
    return working_directory;
}

void SendHandler(VThread *t, ThreadMsg *msg)
{
    const UserMsg *packet = static_cast<const UserMsg *>(msg->msg);
    for (int i = 0; i < num_of_clients; ++i)
    {
        if (write(clients[i].sockfd, packet, sizeof(*packet)) < 0)
        {
            cout << t->get_thread_name() << ": " << strerror(errno) << endl;
            kill(getpid(), SIGINT);
        }
    }
}
void *RecvHandler(void *arg)
{
    int client_idx = *(int *)arg;
    int sockfd = get_sockfd(client_idx);
    int bytecount = 0;
    double buffer[100];
    UserMsg *packet = new UserMsg;
    int ret_val = 0;

    while (1)
    {
        if ((bytecount = recv(sockfd, packet, sizeof(UserMsg), MSG_WAITALL)) == -1)
        {
            fprintf(stderr, "Error receiving data %d\n", errno);
            syslog(LOG_ERR, "Error receiving data %d\n", errno);
        }
        else if (bytecount == 0)
        {
            printf("connection fail\n");
            break;
        }
        for (int i = 0; i < bytecount; i++)
        {
            printf("_%d", packet[i]);
        }
        //syslog(LOG_ERR, "\npacket->cmd_code :%d", packet->cmd_code); //added by hyo
        printf("packet->cmd_code :%s\n", packet->result_msg); //added by hyo
        printf("seq_no: %d, msg.paramId : %d, msg.paramValue : %lf\n", packet->seq_no, packet->param_id, packet->param_val);
        if (packet->cmd_code)
        {
            ret_val = parseMessage(packet);
            sendAck(packet, ret_val);
        }
    }
    pthread_exit((void *)2);
}
void createThreads(int client_idx)
{
    printf("createThread Func\n");
    if (get_cfParam("Main.use_socket") == 1)
    {
        if (pthread_create(&recv_thread[num_of_clients], NULL, &RecvHandler, (void *)&client_idx) < 0)
        {
            printf("error: pthread_create_recv(): %s\n", strerror(errno));
            shutDown();
        }
    }
    if (is_first_connection)
    {
        if (get_cfParam("Main.use_socket") == 0)
        {
            if (pthread_create(&read_thread, NULL, &read_thread_handler, (void *)&client_idx) < 0)
            {
                printf("error: pthread_create_read(): %s\n", strerror(errno));
                shutDown();
            }
        }
        if (get_cfParam("Main.use_ecat"))
            initEcatModule();
        if (get_cfParam("Main.use_obd"))
            initObdModule();
        if (get_cfParam("Main.use_can"))
            initCanModule(); //CAN!!
        if (get_cfParam("Main.use_hvi"))
            initHviModule();

        is_first_connection = false;
    }
    ++num_of_clients;
}
int main(int argc, char *argv[])
{

    sigset_t waitmask;
    int c, flag_help = 0;
    struct in_addr server_ip;
    struct sockaddr_in sockaddr;
    char *ipaddr;
    int portno;
    int halfsd, fullsd; // socket descriptors
    int retval;

    while ((c = getopt(argc, argv, "hi:p:")) != -1)
    {
        switch (c)
        {
        case 'h':
            flag_help = 1;
            break;
        case 'i':
            //memcpy(ipaddr, optarg, strlen(optarg));
            break;
        case 'c':
            portno = atoi(optarg);
            break;
        default:
            printf("unknown option : %c\n", optopt);
            break;
        }
    }

    if (flag_help == 1)
    {
        printf("usage: %s [-h] [-i ipaddr] [-p portno] \n", argv[0]);
        exit(1);
    }
    
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        printf("error: signal(): %s\n", strerror(errno));
        exit(1);
    }
    if (signal(SIGTERM, signalHandler) == SIG_ERR)
    {
        printf("error: signal(): %s\n", strerror(errno));
        exit(1);
    }
    sigemptyset(&waitmask);
    sigaddset(&waitmask, SIGUSR1);
    if(readVehicleConfigs("./vehicle/i30.xml") == -1)
    {
        syslog(LOG_ERR,"READ .xml FAILED!\n");
    }

    if (get_cfParam("Main.use_daemon") == 1)
    {
        
        char *cmd;
        if ((cmd = strrchr(argv[0], '/')) == NULL)
            cmd = argv[0];
        else
            cmd++;
        openlog(cmd, LOG_CONS, LOG_DAEMON);

        if (already_running())
        {
            syslog(LOG_ERR, "daemon already running");
            exit(1);
        }
        
    }
    
    printCar();
    printLogo();
    if (get_cfParam("Main.use_socket"))
    {
        server_ip.s_addr = get_cfParam("Main.ip_addr");
        ipaddr = inet_ntoa(server_ip);
        portno = get_cfParam("Main.ip_port");

        printf("server address = %s:%d\n", ipaddr, portno);

        if ((halfsd = startupServer(ipaddr, portno)) < 0)
        {
            shutDown();
            exit(1);
        }

        initClients();

        while (1)
        {
            int len = sizeof(sockaddr);
            fullsd = accept(halfsd, (struct sockaddr *)&sockaddr, (socklen_t *)&len);
            if (fullsd < 0)
            {
                printf("error : accept() : %s\n", strerror(errno));
                shutDown();
                break;
            }

            printf("Connected\n");

            if (num_of_clients == MAX_CLIENTS)
            {
                printf("error: max clients reached\n");
                close(fullsd);
                sleep(60); //wait for a thread to exit for 1minute
                continue;
            }

            addClient(num_of_clients, fullsd, sockaddr);
            client_idx = findClientByID(num_of_clients);

            createThreads(client_idx);
            printf("main client_idx : %d\n", client_idx);
            send_thread[client_idx] = new VThread("send_thread", client_idx, SendHandler, exitHandler);
            send_thread[client_idx]->CreateThread();
        }

        if (findEmptyClient() == -1)
        {
            sigsuspend(&waitmask);
        }
    }

    else
    {
        printf("not connecting Autoware\n");

        createThreads(client_idx);

        if (findEmptyClient() == -1)
        {
            sigsuspend(&waitmask);
        }

    }
    return 0;
}

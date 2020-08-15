#include <iostream>
#include <string>
#include <map>
#include <cstring>
#include <pthread.h>
#include <utility>

using namespace std;

map<string, int> cfParam;
map<pair<string, string>, map<string, double> > rtParam;

pthread_mutex_t mutex_lock = PTHREAD_MUTEX_INITIALIZER;

//********** Property **********
//**  0 = Dynamic Parameter
//**  1 = Static Parameter
//**  2 = Command Parameter
//******************************

void add_rtParam(pair<string, string> p, char *name, char *nickname,
                   map<string, double> m, int value, int timestamp, int version, int property)
{
    p = make_pair(name,nickname);

    m.insert(pair<string, double>("value", value));
    m.insert(pair<string, double>("timestamp", timestamp));
    m.insert(pair<string, double>("version", version));
    m.insert(pair<string, double>("property", property));

    rtParam.insert({p,m});
}



int get_rtParam_int(char *name, char *field)
{
    map<pair<string, string>, map<string, double> >::iterator m;
    map<string, double> m1;

    for(m = rtParam.begin(); m != rtParam.end(); m++)
    {
        if((strcmp(name, (m->first).first.c_str()) &&
        strcmp(name, (m->first).second.c_str())) == 0)
        {
            pthread_mutex_lock(&mutex_lock);
            m1 = m->second;
            pthread_mutex_unlock(&mutex_lock);
            return m1.find(field)->second;
        }
    }

    printf("No matching rtParam(%s)\n", name);
    return -1;
}
double get_rtParam_double(char *name, char *field)
{
    map<pair<string, string>, map<string, double> >::iterator m;
    map<string, double> m1;

    for(m = rtParam.begin(); m != rtParam.end(); m++)
    {
        if((strcmp(name, (m->first).first.c_str()) &&
        strcmp(name, (m->first).second.c_str())) == 0)
        {
            pthread_mutex_lock(&mutex_lock);
            m1 = m->second;
            pthread_mutex_unlock(&mutex_lock);
            return m1.find(field)->second;
        }
    }

    printf("No matching rtParam(%s)\n", name);
    return -1;
}


int set_rtParam(char *name, char *field, double val)
{
    map<pair<string, string>, map<string, double> >::iterator m;
    for(m = rtParam.begin(); m != rtParam.end(); m++)
    {

        if((strcmp(name, (m->first).first.c_str()) &&
        strcmp(name, (m->first).second.c_str())) == 0)
        {
            if (strcmp(field, "value") == 0)
            {
                int timestamp = get_rtParam_int(name, "timestamp");
                if (timestamp == -1)
                    return -1;
                set_rtParam(name, "timestamp", ++timestamp);
            }

            pthread_mutex_lock(&mutex_lock);
            (m->second).erase(field);
            (m->second).insert(pair<string, double>(field, val));
            pthread_mutex_unlock(&mutex_lock);
            return 1;
        }
    }
    return -1;
    
}


int get_cfParam(char *name)
{
    map<string, int>::iterator v;
    v = cfParam.find(name);
    if (v != cfParam.end())
    {
        return v->second;
    }
    printf("\nNo matching cfParam(%s)!!\n", name);
    return -1;
}

// void classify_rtParam()
// {
//     map<string, map<string, double>>::iterator v;
//     map<string, double> v1;
//     int p0 = 0, p1 = 0, p2 = 0;
//     for (v = rtParam.begin(); v != rtParam.end(); ++v)
//     {
//         v1 = v->second;
//         switch ((int)v1.find("property")->second)
//         {
//         case 0:
//             ++p0;
//             break;
//         case 1:
//             ++p1;
//             break;
//         case 2:
//             ++p2;
//             break;
//         }
//     }
//     cfParam.insert(pair<string, double>("dynamic_param.count", p0));
//     cfParam.insert(pair<string, double>("static_param.count", p1));
//     cfParam.insert(pair<string, double>("command_param.count", p2));
// }
//parsing xml file.

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <map>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <syslog.h> 

using namespace std;

extern map<string, int> cfParam;
extern void open_log(char *fname);
extern int get_cfParam(char *name);
/*
 *To compile this file using gcc you can type
 *gcc `xml2-config --cflags --libs` -o xmlexample libxml2-example.c
 */
void set_Throttle_convert_position()
{
    double physical_range_pos = abs(get_cfParam("Throttle.max_position") - get_cfParam("Throttle.min_position"));
    double virtual_range_pos = abs(get_cfParam("Throttle.virtual_max_position") - get_cfParam("Throttle.virtual_min_position"));
    cfParam.insert(pair<string, int>("Throttle.convert_position", physical_range_pos / virtual_range_pos));
}
void set_Brake_convert_position()
{
    double physical_range_pos = abs(get_cfParam("Brake.max_position") - get_cfParam("Brake.min_position"));
    double virtual_range_pos = abs(get_cfParam("Brake.virtual_max_position") - get_cfParam("Brake.virtual_min_position"));
    cfParam.insert(pair<string, int>("Brake.convert_position", physical_range_pos / virtual_range_pos));
}
void set_Steer_convert_position()
{
    double physical_range_pos = abs(get_cfParam("Steerwheel.max_position") - get_cfParam("Steerwheel.min_position"));
    double virtual_range_pos = abs(get_cfParam("Steerwheel.virtual_max_position") - get_cfParam("Steerwheel.virtual_min_position"));
    cfParam.insert(pair<string, int>("Steerwheel.convert_position", physical_range_pos / virtual_range_pos));
}
static void
getNode_PressThrottle_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"lowerbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Throttle.min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"upperbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Throttle.max_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtuallowerbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Throttle.virtual_min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtualupperbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Throttle.virtual_max_position", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_PressBrake_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"lowerbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Brake.min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"upperbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Brake.max_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"startpos")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Brake.start_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtuallowerbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Brake.virtual_min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtualupperbound")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Brake.virtual_max_position", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_SteerWheel_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"leftmost")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Steerwheel.max_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"origin")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Steerwheel.start_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rightmost")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Steerwheel.min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtualleftmost")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Steerwheel.virtual_min_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"virtualrightmost")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Steerwheel.virtual_max_position", atoi(s)));
                xmlFree(s);
            }
        }
    }
}

static void
getNode_ShiftGearStick_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"PmodePos")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Gearstick.park_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"RmodePos")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Gearstick.reverse_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"NmodePos")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Gearstick.neutral_position", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"DmodePos")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Gearstick.drive_position", atoi(s)));
                xmlFree(s);
            }
        }
    }
}

static void
getNode_PullOver_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"jerk")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Pullover.jerk", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"pospertick")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Pullover.position_per_tick", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"errmargin")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Pullover.error_margin", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_HomingPedals_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"margin")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HomingPedals.tolerance", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_CenterLidar_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"mindegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.center_min_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"maxdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.center_max_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_LeftLidar_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"mindegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.left_min_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"maxdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.left_max_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_RightLidar_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {

            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"mindegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.right_min_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"maxdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.right_max_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_DrivingMode_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.DriveMode.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.DriveMode.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.DriveMode.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_ParkingMode_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.ParkMode.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.ParkMode.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.ParkMode.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_MappingMode_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.MappingMode.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.MappingMode.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.MappingMode.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_HomingMode_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.HomeMode.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.HomeMode.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.HomeMode.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_Ouster_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Ouster.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Ouster.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Ouster.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_Velodyne_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Velodyne.center_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ldegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Velodyne.left_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("PoseLidar.Velodyne.right_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_PoseLidar_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"posperdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Lidar.position_per_degree", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"centerlidar")))
            {
                getNode_CenterLidar_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"leftlidar")))
            {
                getNode_LeftLidar_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rightlidar")))
            {
                getNode_RightLidar_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"homingmode")))
            {
                getNode_HomingMode_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ouster")))
            {
                getNode_Ouster_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"velodyne")))
            {
                getNode_Velodyne_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"mappingmode")))
            {
                getNode_MappingMode_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"drivingmode")))
            {
                getNode_DrivingMode_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"parkingmode")))
            {
                getNode_ParkingMode_children(cur_node->children);
            }
            
            
        }
    }
}
static void
getNode_SelfTest_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"throttlepospertick")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("SelfTest.Throttle.position_per_tick", atoi(s)));
                xmlFree(s);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"brakepospertick")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("SelfTest.Brake.position_per_tick", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_Motion_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"pressthrottle")))
            {
                getNode_PressThrottle_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"pressbrake")))
            {
                getNode_PressBrake_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"steerwheel")))
            {
                getNode_SteerWheel_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"shiftgearstick")))
            {
                getNode_ShiftGearStick_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"pullover")))
            {
                getNode_PullOver_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"homingpedals")))
            {
                getNode_HomingPedals_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"poselidar")))
            {
                getNode_PoseLidar_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"selftest")))
            {
                getNode_SelfTest_children(cur_node->children);
            }
        }
    }
}
static void
getNode_OBDaccelPID_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kp")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.accel_gain_p", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ki")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.accel_gain_i", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kd")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.accel_gain_d", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_OBDdecelPID_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kp")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.decel_gain_p", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ki")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.decel_gain_i", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kd")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.OBD.decel_gain_d", atoi(s)));
                xmlFree(s);
            }
        }
    }
}


static void
getNode_CANaccelPID_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kp")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.accel_gain_p", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ki")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.accel_gain_i", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kd")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.accel_gain_d", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_CANdecelPID_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kp")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.decel_gain_p", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ki")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.decel_gain_i", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"kd")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.CAN.decel_gain_d", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_CruiseControl_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"obdaccelpid")))
            {
                getNode_OBDaccelPID_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"obddecelpid")))
            {
                getNode_OBDdecelPID_children(cur_node->children);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"canaccelpid")))
            {
                getNode_CANaccelPID_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"candecelpid")))
            {
                getNode_CANdecelPID_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"velplusmargingain")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.accel_velocity_tolerance", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"velminusmargingain")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.decel_velocity_tolerance", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"tvelmin")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.min_target_velocity", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"tvelmax")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.max_target_velocity", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Ierrbase")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.accumulated_error_base", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Ierrincr")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("CruiseControl.accumulated_error_delta", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_SteeringControl_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"wheelbase")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("SteerControl.wheel_base", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"minradius")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("SteerControl.min_radius", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"posperdegree")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("SteerControl.position_per_degree", atoi(s)));
                xmlFree(s);
            }
        }
    }
}

static void
getNode_Controller_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"cruisecontrol")))
            {
                getNode_CruiseControl_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"steeringcontrol")))
            {
                getNode_SteeringControl_children(cur_node->children);
            }
        }
    }
}

static void
getNode_EthercatCyclic_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Motion")))
            {
                getNode_Motion_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Controller")))
            {
                getNode_Controller_children(cur_node->children);
            }
        }
    }
}
static void
getNode_EthercatManager_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"slavecnt")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.num_motors", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"throttleid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_throttle", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"brakeid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_brake", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"wheelid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_steer", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"gearid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_gearstick", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"centerlidarid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_lidar_center", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"leftlidarid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_lidar_left", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rightlidarid")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("ECAT.motor_id_lidar_right", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_MAIN_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"usedaemon")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_daemon", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"connectclient")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_socket", atoi(s)));
                xmlFree(s);
            }

            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"connectobd")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_obd", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"connectecat")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_ecat", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"connectgway")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_can", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"connecthvi")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_hvi", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"useprotobuf")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.use_protobuf", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ipaddress")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.ip_addr", inet_addr(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ipport")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("Main.ip_port", atoi(s)));
                xmlFree(s);
            }
        }
    }
}

static void
getNode_LOG_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;
    int log_param = 0;
    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"logparams")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                log_param = atoi(s);
                cfParam.insert(pair<string, int>("log", log_param));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"filename")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                if (log_param == 1)
                    open_log(s);
                xmlFree(s);
            }
        }
    }
}
static void
getNode_EStopButton_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ingpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_btn_seat_gpio", atoi(s)));
                xmlFree(s);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"outcgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_btn_center_gpio", atoi(s)));
                xmlFree(s);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"outlgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_btn_left_gpio", atoi(s)));
                xmlFree(s);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"outrgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_btn_right_gpio", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_EStopReceiver_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rungpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_remote_run_gpio", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"pausegpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.estop_remote_pause_gpio", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_Buzzer_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"frontgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.buzzer_front_gpio", atoi(s)));
                xmlFree(s);
            }
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"reargpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.buzzer_rear_gpio", atoi(s)));
                xmlFree(s);
            }
        }
    }
}
static void
getNode_Signtower_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"lredgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.signtower_left_red_gpio", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"lgreengpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.signtower_left_green_gpio", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rredgpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.signtower_right_red_gpio", atoi(s)));
                xmlFree(s);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"rgreengpio")))
            {
                xmlChar *s = xmlNodeGetContent(cur_node);
                cfParam.insert(pair<string, int>("HVI.signtower_right_green_gpio", atoi(s)));
                xmlFree(s);
            }
        }
    }
}

static void
getNode_HVIManager_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"EStopButton")))
            {
                getNode_EStopButton_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"EStopReceiver")))
            {
                getNode_EStopReceiver_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Buzzer")))
            {
                getNode_Buzzer_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Signtower")))
            {
                getNode_Signtower_children(cur_node->children);
            }
        }
    }
}
static void
getNode_ICHTHUS_children(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Log")))
            {
                getNode_LOG_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"Main")))
            {
                getNode_MAIN_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"EthercatManager")))
            {
                getNode_EthercatManager_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"EthercatCyclic")))
            {
                getNode_EthercatCyclic_children(cur_node->children);
            }
            else if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"HVIManager")))
            {
                getNode_HVIManager_children(cur_node->children);
            }
        }
    }
}
static void
getNode_Vehicle(xmlNode *node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = node; cur_node; cur_node = cur_node->next)
    {
        if (cur_node->type == XML_ELEMENT_NODE)
        {
            if ((!xmlStrcmp(cur_node->name, (const xmlChar *)"ICHTHUS")))
            {
                getNode_ICHTHUS_children(node->children);
            }
        }
    }
}

void printConfigs() // qjin : need to write instruction
{
    map<string, int>::iterator i;

    cout << "\n****************************************************\n**" << endl;
    for (i = cfParam.begin(); i != cfParam.end(); ++i)
    {
        cout << "**   " << i->first << " = " << i->second << "\n";
    }
    cout << "**\n****************************************************" << endl;

    syslog(LOG_ERR,"\n****************************************************\n**\n");
    for (i = cfParam.begin(); i != cfParam.end(); ++i)
    {
        syslog(LOG_ERR,"**   %s = %d\n",i->first.c_str(), i->second);
    }
    syslog(LOG_ERR,"**\n****************************************************\n");
}

int readVehicleConfigs(char *xml_file)
{
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;

    /*
   * this initialize the library and check potential ABI mismatches
   * between the version it was compiled for and the actual shared
   * library used.
   */
    LIBXML_TEST_VERSION

    /* parse the file and get the DOM */
    doc = xmlReadFile(xml_file, NULL, 0);

    if (doc == NULL)
    {
        printf("error: could not parse file %s\n", xml_file);
        return -1;
    }

    /* get the root element node */
    root_element = xmlDocGetRootElement(doc);

    /* parse all the element nodes */
    getNode_Vehicle(root_element);
    set_Throttle_convert_position();
    set_Brake_convert_position();
    set_Steer_convert_position();
    printConfigs();

    /* free the document */
    xmlFreeDoc(doc);

    /*
   * Free the global variables that may
   * have been allocated by the parser.
   */
    xmlCleanupParser();
    return 0;
}

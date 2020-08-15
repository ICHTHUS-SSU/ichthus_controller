#include <string.h>
using namespace std;

typedef struct UserMsg {
    UserMsg() { seq_no = 0; ack_no =0; cmd_code = 0; param_id = 0; param_val = 0; result_code = 0;}
    UserMsg(int s_no, int a_no, int c_code, int p_id, double p_val, int r_code, char *s) {seq_no = s_no; ack_no = a_no; cmd_code = c_code; param_id = p_id; param_val = p_val; result_code = r_code; strcpy(result_msg, s); }
    int seq_no;
    int ack_no;
    int cmd_code;//0 means get / 1 means set
    int param_id;//0 means target velocity/ 1 means target omega//temporally
    double param_val;//used as reply for get
    int result_code;//result of cmd(0 means success)
    char result_msg[100];//infor/warning/error msg
};//added by hyo

#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <regex>
#include <vector>
#include <thread>
#include<cstdio>
#include "ichthus_controller/con_msg.h"



using namespace std;
using namespace ros;

// object used to call ros service
ros::ServiceClient client;

void printvelocity(void){
  while(1){
  printf("\033[%d;%df", 1, 0);
  //fflush(stdout);
  cout<<"------------------------------------velocity-------------------------------------\n\n\n";
  fflush(stdout);
  sleep(1);
  }
}
// list of basic command formats
vector<string> basic_fmt_list;
int num_basic_fmts;

// list of macro command formats
vector<string> macro_fmt_list;
vector<vector<string> > macro_def_list;
int num_macro_fmts;

// list of commands executed
vector<string> cmd_hist_list;
int num_cmd_hist;

void readParams(NodeHandle& nh) {
  string name, format;
  vector<string> defini;

  // read basic formats
  num_basic_fmts = 0;
  while (1) {
    name = "basic_fmt" + to_string(num_basic_fmts);
    if(!nh.getParam("/" + name, format))
      break;
    basic_fmt_list.push_back(format);
    num_basic_fmts++;
  }

  // read macro formats
  num_macro_fmts = 0;
  while (1) {
    name = "macro_fmt" + to_string(num_macro_fmts);
    if(!nh.getParam("/" + name, format))
      break;

    name = "macro_def" + to_string(num_macro_fmts);
    if(!nh.getParam("/" + name, defini))
      break;
    
    macro_fmt_list.push_back(format);
    macro_def_list.push_back(defini);
    num_macro_fmts++;
  }
}

void printParams() {
  int i;
  vector<string>::iterator it, kt;
  vector<vector<string> >::iterator jt;

  for (i=0, it=basic_fmt_list.begin(); it != basic_fmt_list.end(); ++it, ++i)
    cout << "basic_fmt" << i << ": " << *it << endl;

  for (i=0, it=macro_fmt_list.begin(), jt=macro_def_list.begin();
       it != macro_fmt_list.end();
       ++it, ++jt, ++i) {
    cout << "macro_fmt" << i << ": " << *it << endl;
    cout << "macro_def" << i << ": ";

    for (kt=(*jt).begin(); kt != (*jt).end(); ++kt)
      cout << *kt << ", ";
    cout << endl;
  }
}

void callService(string cmd, string arg1, string arg2) {
  ichthus_controller::con_msg srv;

  srv.request.cmd  = cmd.c_str();
  srv.request.arg1 = arg1.c_str();
  srv.request.arg2 = arg2.c_str();

  if (client.call(srv))
    cout << "ret: " << arg1.c_str()  << srv.response.res.c_str() << endl;
  else
    cout << "ret: fail to call!!" << endl;
}

void handler_help() {
  printParams();
}

void handler_quit() {
  cout << "ichthus_controller_client terminated!!" << endl;
  exit(0);
}

void handler_history() {
  for(int i=0; i < num_cmd_hist; i++)
    cout << " " << to_string(i+1) + ". " << cmd_hist_list[i] << endl;
  cout << endl;
}

void executeCommand(smatch& match) {
  int n = match.size();
  cout << "match[ ";
  for (int i=0; i < n-1; i++)
    cout << "\"" << match[i].str() << "\", ";
  cout << "\"" << match[n-1].str() << "\" ]" <<  endl;

  switch (match.size()) {
  case 2: // a command of a single regex
    if (match[1] == "help")
      handler_help();
    else if (match[1] == "quit")
      handler_quit();
    else if (match[1] == "history")
      handler_history();
    else
      callService(match[1], "", "");
    break;
  case 3: // a command of two regex's
    callService(match[1], match[2], "");
    break;
  case 4: // a command of three regex's
    callService(match[1], match[2], match[3]);
    break;
  default:
    cout << "### a wrong command format" << endl;
  }
}

void doLoop() {
  string line;
  smatch match;
  int i;
  vector<string>::iterator it;
  
  regex *basic_regex[num_basic_fmts];
  for (int i=0; i < num_basic_fmts; i++)
    basic_regex[i] = new regex(basic_fmt_list[i]);

  regex *macro_regex[num_macro_fmts];
  for (int i=0; i < num_macro_fmts; i++)
    macro_regex[i] = new regex(macro_fmt_list[i]);

  //thread t1(printvelocity);
  
  while (ros::ok()) {
    cout << "$$$ ";
    getline(cin, line);
  line_read:    
    // try to match line with macro_regex[]
    for (i=0; i < num_macro_fmts; i++) {
      if (regex_match(line, match, *macro_regex[i])) {
	cmd_hist_list.push_back(line);
	num_cmd_hist++;

	for (it=macro_def_list[i].begin(); it != macro_def_list[i].end(); ++it) {
	  for (int j=0; j < num_basic_fmts; j++) {
	    if (regex_search(*it, match, *basic_regex[j])) {
	      executeCommand(match);
	      break;
	    }
	  }
	}

	break; // break out the outermost for-loop
      }
    } // for (i=0; i < num_macro_fmts; i++)

    if(i != num_macro_fmts)
      continue;  // go to the beginning of the while-loop
    
    // try to match line with basic_regex[]
    for (i=0; i < num_basic_fmts; i++) {
      if (regex_match(line, match, *basic_regex[i])) {
	if (match.size() == 3 && match[1] == "!") { // command history lookup
	  string arg = match[2].str();
	  int idx = atoi(arg.c_str()); // one-based index
	  if (idx < 1 || idx > num_cmd_hist) {
	    cout << "### a wrong history index" << endl;
	    break;
	  }
	  line = cmd_hist_list[idx-1]; // zero-based index
	  goto line_read;
	}
	cmd_hist_list.push_back(line);
	num_cmd_hist++;
	executeCommand(match);
	break;
      }
    } // for (i=0; i < num_basic_fmts; i++)
  } // while (ros::ok())
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ichthus_controller_client");
  ros::NodeHandle nh;
  readParams(nh);
  //printParams();
  client = nh.serviceClient<ichthus_controller::con_msg>("ichthus_controller");
  doLoop();
  //system("clear");
  return 0;
}
 

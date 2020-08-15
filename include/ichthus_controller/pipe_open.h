/*
 * Copyright 2019 Soongsil University. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __PIPE_OPEN_H__
#define __PIPE_OPEN_H__

/////////////////////////////////////////////////////////////////////
// We borrowed the following code from
// https://stackoverflow.com/questions/45202379/how-does-popen-work-and-how-to-implement-it-into-c-code-on-linux
/////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <string>
#include <cstring>   // sterror
#include <cstdio>  // popen, FILE, fgets, fputs
#include <cassert>

using namespace std;  

namespace pipe_open
{
  class PipeOpen {
  protected:
    FILE* m_fp;
    string m_cmd;

  public:
    PipeOpen(void) : m_fp(0) {}
    virtual ~PipeOpen(void) {
      if (m_fp) (void) close();
      m_fp = 0;
    }

    // on success: 0 == return.size(), else returns error msg
    string close() {
      stringstream err_ss;

      do {
	assert(m_fp != 0);
	// pclose() returns the term status of the shell cmd
	// otherwise -1 and sets errno.
	int32_t stat = ::pclose(m_fp);
	int errno_copy = errno;
	if (stat != 0) {
	  err_ss << "\n  pipe_open::close() errno " << errno_copy
		 << "  " << strerror(errno_copy) << endl;
	  break;
	}
	m_fp = 0;
      } while (0);
      
      return(err_ss.str());
    } 

  }; // class pipe_open

  class PipeOpenRead : public PipeOpen {
  public:
    PipeOpenRead(void) {}
    string open(string cmd) {
      stringstream err_ss;
      assert(cmd.size() > 0);
      assert(m_fp == 0);
      m_cmd = cmd;

      //cout << "\n  pipe_open_read::open(cmd): cmd: '" << m_cmd << "'\n" << endl;
      m_fp = ::popen(m_cmd.c_str(), "r");

      int errno_copy = errno;
      if(m_fp == 0) {
	err_ss << "\n  pipe_open_read::open(" << m_cmd
	       << ") popen() errno "   << errno_copy
	       << "  " << strerror(errno_copy) << endl;
      }
      return (err_ss.str());
    }

    string read(stringstream& out_ss) {
      const int BUFF_SIZE = 2048;
      stringstream err_ss;
      do {
	if(m_fp == 0) {
	  err_ss << "\n  ERR: pipe_open_read::log(out_ss) - pipe closed";
	  break;
	}
	size_t blank_lines = 0;
	do {
	  char buff[BUFF_SIZE] = { 0 };
	  for (int i=0; i < BUFF_SIZE; ++i)
	    assert(buff[i] == 0);
	  char* stat = fgets(buff, BUFF_SIZE, m_fp);
	  assert((stat == buff) || (stat == 0));
	  int errno_copy = errno;
	  if(feof(m_fp)) break;
	  if(ferror(m_fp)) {
	    err_ss << "err: fgets() with ferror: " << strerror(errno_copy);
	    break;
	  }

	  if(strlen(buff)) out_ss << buff;
	  else blank_lines++;
	} while (1);

	if(blank_lines)
	  out_ss << "\n" << "discarded blank lines: " << blank_lines << endl;
      } while (0);

      return (err_ss.str());
    }
  }; // class pipe_open_read

 class PipeOpenWrite : public PipeOpen {
 public:
   PipeOpenWrite(void) {}
   string open(string cmd) {
     stringstream err_ss;
     assert (cmd.size() > 0);
     assert (m_fp == 0);
     m_cmd = cmd;

     do {
      //cout << "\n  pipe_open_write::open(cmd): cmd: '" << m_cmd << "'\n" << end
       m_fp = ::popen(m_cmd.c_str(), "w");
       int errno_copy = errno;
       if(m_fp == 0) {
	 err_ss << "\n  pipe_open_write::open(" << m_cmd
		<< ") popen() errno "        << errno_copy
		<< "  "  << strerror(errno_copy)  << endl;
	 break;
       }
     } while (0);

     return (err_ss.str());
   }
   void write(stringstream& in_ss) {} // not implemented yet
 }; // class pipe_open_write
}

#endif // __PIPE_OPEN_H__

/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  pthread_detach.cpp -- Thread processes that ran are detached

  With the Posix Threads coroutine package, a thread process that has been
  resumed must run on a detached pthread, so the thread's resources are
  released when it terminates and not only when its process object is
  deleted. The check needs pthread_getattr_np() and is done with glibc
  only; elsewhere the test only runs the processes.

  Original Author: Mayur Kubavat, 2026-11-16

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include <systemc>
#include <vector>

#if defined(SC_USE_PTHREADS) && defined(__GLIBC__)
#include <pthread.h>
#define CHECK_DETACH_STATE 1
#endif

using namespace sc_core;

static bool self_detached()
{
#ifdef CHECK_DETACH_STATE
    pthread_attr_t attr;
    int state = PTHREAD_CREATE_JOINABLE;
    if ( pthread_getattr_np( pthread_self(), &attr ) == 0 )
    {
        pthread_attr_getdetachstate( &attr, &state );
        pthread_attr_destroy( &attr );
    }
    return state == PTHREAD_CREATE_DETACHED;
#else
    return true;
#endif
}

static void report( const char* what )
{
    std::cout << sc_time_stamp() << " " << what << ": "
              << ( self_detached() ? "detached" : "joinable" ) << std::endl;
}

static void spawned()
{
    report( "spawned thread" );
}

SC_MODULE( top )
{
    std::vector<sc_process_handle> handles;

    SC_CTOR( top )
    {
        SC_THREAD( static_thread );
    }

    void static_thread()
    {
        report( "static thread" );
        wait( 1, SC_NS );
        report( "static thread after wait" );
        for ( int i = 0; i < 3; ++i )
        {
            handles.push_back( sc_spawn( &spawned ) );
            wait( 1, SC_NS );
        }
    }
};

int sc_main( int, char*[] )
{
    top t( "top" );
    sc_start();
    std::cout << "processes kept: " << t.handles.size() << std::endl;
    return 0;
}

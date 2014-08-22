/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#ifndef __WORKSPACEEVTHREAD_H__
#define __WORKSPACEEVTHREAD_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/ctrl/math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdarg.h>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;


class workspaceEvThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the thread:
    string name;
    // Translational Tolerance:   
    double translationalTol;
    // Orientational Tolerance:
    double orientationalTol;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int    cnt;                     // Counter that handles the cycling through the positions
    int    step;                    // Counter that manages the state machine
    bool   isJobDone;
    string src_mode;
    string homePath;
    string outputFile;

    iKinChain     chain;
    iKinIpOptMin *slv;

    vector<Vector> poss2Expl;
    vector<Vector> oris2Expl;
    vector<int>    reachability;

    /**
     * Explores the workspace by iteratively span all the joints and store the position into a suitable variable.
     * @param  jnt the link from which the exploration starts (usually 0)
     * @return     true/false if success/failure
     */
    bool exploreWorkspace();

    /**
     * Saves the workspace on file. Filename is given by outputFile;
     * @return     true/false if success/failure
     */
    bool saveWorkspace();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    workspaceEvThread(int _rate, int _v, string _n, double _tT, double _oT,
                      const iKinChain &_c, const vector<Vector> &_p2E,
                      const vector<Vector> &_o2E, string _oF);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
     * Checks if the thread has finished the job.
     * @return     true/false if finished/still going on
     */
    bool checkJobDone() { return isJobDone; };
};

#endif

// empty line to make gcc happy

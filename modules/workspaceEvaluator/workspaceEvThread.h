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
#include <cmath>

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
    string name;              // Name of the thread
    string src_mode;
    string eval_mode;         // Evaluation mode
    string expl_mode;         // Exploration mode
    string outputFile;        // Output file

    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;

    double granP;
    int    resolJ;            // Resolution in the joint space

    double rate;              // Rate of the thread
    double XYZTol;            // Translational Tolerance

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int    cnt;                     // Counter that handles the cycling through the positions
    int    step;                    // Counter that manages the state machine
    bool   isJobDone;
    double timeStart;
    double timeEnd;

    iKinChain     chain;
    iKinIpOptMin *slv;

    vector<Vector> explVec;
    vector<double> reachability;

    /**
     * Explores the workspace by iteratively span all the joints and store the position into a suitable variable.
     * @param  jnt the link from which the exploration starts (usually 0)
     * @return     true/false if success/failure
     */
    bool exploreWorkSpace();

    bool exploreOperationalSpace();

    /**
    * Explores the workspace by iteratively span all the joints and store the position into a suitable variable.
    * @param jnt the link from which the exploration starts (usually 0)
    * @return true/false if success/failure
    */
    bool exploreJointSpace(const int &jnt=0);

    /**
     * Computes the manipulability for the iKinChain to be stored into the reachability vector.
     * @return the manipulability value in the current configuration.
     */
    double computeManipulability();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;
    
public:
    // CONSTRUCTOR
    workspaceEvThread(int _rate, int _v, string _n, double _tT,
                      const iKinChain &_c, const vector<Vector> &_p2E,
                      string _oF, string _sM, string _eVM, string _eXM,
                      double _gP, int _rJ);

    // COPY CONSTRUCTOR
    workspaceEvThread(const workspaceEvThread &_wET);

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

    /**
     * Saves the workspace on file. Filename is given by outputFile;
     * @return     true/false if success/failure
     */
    bool saveWorkspace();

    /**
     * Get Functions
     */
    int getVerbosity()   const { return verbosity; };
    string getName()     const { return name; };
    iKinChain getChain() const { return chain; };
    double getRate()     const { return rate; };
    double getXYZTol()   const { return XYZTol; };
    double getAdvancement()    { return 100.0*cnt/explVec.size(); };

    string getOutputFile() const { return outputFile; };

    vector<Vector> getExplVec() const { return explVec; };
    string getSrcMode()  const { return src_mode;  };
    string getEvalMode() const { return eval_mode; };
    string getExplMode() const { return expl_mode; };

    double getGranP()    const { return granP;  };
    int    getResolJ()   const { return resolJ; };
};

#endif

// empty line to make gcc happy

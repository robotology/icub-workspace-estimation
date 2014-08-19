/* ICUB WORKSPACE EVALUATOR v. 1.0
 * Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Alessandro Roncone
 * email:   alessandro.roncone@iit.it
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
*/

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <string>
#include <vector>
#include <stdarg.h>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

using namespace iCub::iKin;

using namespace std;

class workspaceEvaluator: public RFModule 
{
private:
    string name;
    string src_mode;
    double rate;
    int    verbosity;
    bool   isJobDone;

    iKinChain *chain;
    double granularity;
    double increment;

    Vector foo;

public:
    workspaceEvaluator()
    {
        foo.resize(3,0.0);
        isJobDone = 0;
    }

    bool configure(ResourceFinder &rf)
    {
        name        = "workspaceEvaluator"; // name
        verbosity   = 0;                    // verbosity
        rate        = 0.5;                  // rate
        src_mode    = "test";               // src_mode
        granularity = 0.01;
        increment   = CTRL_DEG2RAD*1;

        //******************************************************
        //********************** CONFIGS ***********************

        //********************* NAME *******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                printMessage(0,"Module name set to %s\n",name.c_str());
            }
            else printMessage(0,"Module name set to default, i.e. %s\n", name.c_str());
            setName(name.c_str());

        //******************** RATE ********************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt()/1000;
                printMessage(0,"Module working at %gs\n", rate);
            }
            else printMessage(0,"Could not find rate in the config file; using %gs as default.\n",rate);

        //******************** TICKS ********************
            if (rf.check("increment"))
            {
                increment = rf.find("increment").asDouble();
                printMessage(0,"Each joint will be divided into %g increment\n", increment);
            }
            else printMessage(0,"Could not find increment in the config file; using %g as default.\n",increment);

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                printMessage(0,"Module verbosity set to %i\n",verbosity);
            }
            else printMessage(0,"Could not find verbosity option in config file; using %i as default.\n",verbosity);

        //************** SOURCE_MODE **************
            if (rf.check("src_mode"))
            {
                if (rf.find("src_mode").asString() == "test")// || rf.find("src_mode").asString() == "DH" || rf.find("src_mode").asString() == "URDF")
                {
                    src_mode = rf.find("src_mode").asString();
                    printMessage(0,"src_mode set to %s\n",src_mode.c_str());
                }
                else printMessage(0,"src_mode option found but not allowed; using %s as default.\n",src_mode.c_str());
            }
            else printMessage(0,"Could not find src_mode option in the config file; using %s as default.\n",src_mode.c_str());

            configureChain();

        return true;
    }

    bool close()
    {   
        return true;
    }

    double getPeriod()
    {
        return rate;
    }

    bool updateModule()
    {
        if (!isJobDone)
        {
            exploreWorkspace(9);
            saveWorkspace();
            isJobDone = 1;
        }
        else
            printMessage(0,"Finished\n");
        return true;
    }

    /**
     * Explores the workspace by iteratively span all the joints and store the position into a suitable variable.
     * @param  jnt the link from which the exploration starts (usually 0)
     * @return     true/false if success/failure
     */
    bool exploreWorkspace(const int &jnt=0)
    {
        double min  = (*chain)[jnt].getMin();
        double max  = (*chain)[jnt].getMax();

        // The spaces are there in order to make some order into the printouts (otherwise its a mess)
        string spaces="";
        for (int i = 0; i < jnt; i++)
        {
            spaces += "  ";
        }

        printMessage(0,(spaces+"Analyzing link #%i : min %g\tmax %g\n").c_str(),jnt,min,max);

        chain->setAng(jnt,min);
        printMessage(1,(spaces+"Link #%i ang set to %g\n").c_str(),jnt,chain->getAng(jnt));

        while (chain->getAng(jnt) < max)
        {
            if (jnt+1 < chain->getN())
            {
                exploreWorkspace(jnt+1);
            }
            else
            {
                printMessage(2,"Links are finished!\n");
            }
            double timeStart = yarp::os::Time::now();
            Matrix H   = chain->getH();
            Vector pos = H.getCol(3).subVector(0,2);
            chain->setAng(jnt,chain->getAng(jnt)+increment);
            storePosition(pos);
            double timeEnd = yarp::os::Time::now();

            printMessage(0,"Elapsed time: %g\n",timeEnd-timeStart);
        }

        return true;
    }

    bool storePosition(const Vector &pos)
    {
        foo = pos;
        return true;
    }

    bool saveWorkspace()
    {
        return true;
    }

    /**
     * Configures the chain according to the src_mode. It can be either a test chain (a classical iCubArm left),
     * a DH file, or an URDF file
     * @return true/false if success/failure
     */
    bool configureChain()
    {
        if (src_mode == "test")
        {
            iCubArm *arm = new iCubArm("left");
            chain = arm->asChain();
            return true;
        }
        else if (src_mode == "DH")
        {
            
        }
        else if (src_mode == "URDF")
        {
            
        }

        return false;
    }
    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const
    {
        if (verbosity>=l)
        {
            fprintf(stdout,"*** %s: ",name.c_str());

            va_list ap;
            va_start(ap,f);
            int ret=vfprintf(stdout,f,ap);
            va_end(ap);

            return ret;
        }
        else
            return -1;
    }

};

/**
* Main function.
*/
int main(int argc, char * argv[])
{
    ResourceFinder moduleRF;
    moduleRF.setVerbose(false);
    moduleRF.setDefaultContext("iCubWorkspace");
    moduleRF.setDefaultConfigFile("workspaceEvaluator.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        cout << endl << "Options:" << endl;
        cout << "   --context   path:  where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from      from:  the name of the .ini file (default workspaceEvaluator.ini)." << endl;
        cout << "   --name      name:  the name of the module (default workspaceEvaluator)." << endl;
        cout << "   --verbosity int:   verbosity level (default 0)." << endl;
        cout << "   --rate      int:   the period used by the module. Default 500ms." << endl;
        cout << "   --increment     int:   the number of increment that span each joint. Default 10." << endl;
        cout << "   --src_mode  mode:  source to use finding the chain (either test, DH, or URDF; default test)." << endl;
        cout << endl;
        return 0;
    }

    workspaceEvaluator workEv;
    return workEv.runModule(moduleRF);
}

// empty line to make gcc happy

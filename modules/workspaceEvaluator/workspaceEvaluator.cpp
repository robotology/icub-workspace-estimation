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
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/ctrl/math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdarg.h>

#include "workspaceEvThread.h"

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class workspaceEvaluator: public RFModule 
{
private:
    vector<workspaceEvThread*>wsEvThreads;
    workspaceEvThread *wSET;
    ResourceFinder rf;
    string name;
    double rate;
    int    verbosity;
    bool   isJobDone;
    double granP;
    double translationalTol;
    double orientationalTol;

    string src_mode;
    string homePath;
    string outputFile;
    string DH_file;
    string URDF_file;

    Matrix wsLims;

    iCubArm      *arm;       // This is used only if src_mode=="test"
    iKinLimb     *limb;      // This is used if src_mode=="DH" || src_mode=="URDF"
    iKinChain    *chain;

    vector<Vector> poss2Expl;
    vector<Vector> oris2Expl;
    vector<double> reachability;

public:
    workspaceEvaluator()
    {
        arm       = 0;
        limb      = 0;
        isJobDone = 0;
        name             = "workspaceEvaluator"; // name
        verbosity        = 0;                    // verbosity
        rate             = 0.5;                  // rate
        granP            = 0.01;                 // spatial granularity
        translationalTol = 5e-3;                 // translational tolerance
        orientationalTol = 20;                   // orientational tolerance
        src_mode         = "test";               // src_mode
        DH_file          = "DH.ini";
        URDF_file        = "URDF.xml";

        // These are the limits of the exploration of the workspace, defined as a 3x2 matrix
        // wsLims = [minX maxX; minY maxY; minZ maxZ]
        wsLims.resize(3,2);
        wsLims.zero();
    }

    bool configure(ResourceFinder &_rf)
    {
        rf=_rf;
        //******************************************************
        //********************** CONFIGS ***********************

        //************************ NAME ************************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                printMessage(0,"Module name set to %s\n",name.c_str());
            }
            else printMessage(0,"Module name set to default, i.e. %s\n", name.c_str());
            setName(name.c_str());

        //************************ RATE ************************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt()/1000;
                printMessage(0,"Module working at %gs\n", rate);
            }
            else printMessage(0,"Could not find rate in the config file; using %gs as default.\n",rate);

        //**************** SPATIAL GRANULARITY *****************
            if (rf.check("granP"))
            {
                granP = rf.find("granP").asDouble();
                printMessage(0,"Granularity will be %g m\n", granP);
            }
            else printMessage(0,"Could not find granP in the config file; using %g as default.\n",granP);

        //************** TRANSLATIONAL TOLERANCE ***************
            if (rf.check("translationalTol"))
            {
                translationalTol = rf.find("translationalTol").asDouble();
                printMessage(0,"Each joint will be divided into %g translationalTol\n", translationalTol);
            }
            else printMessage(0,"Could not find translationalTol in the config file; using %g as default.\n",translationalTol);

        //************** ORIENTATIONAL TOLERANCE ***************
            if (rf.check("orientationalTol"))
            {
                orientationalTol = rf.find("orientationalTol").asDouble();
                printMessage(0,"Each joint will be divided into %g orientationalTol\n", orientationalTol);
            }
            else printMessage(0,"Could not find orientationalTol in the config file; using %g as default.\n",orientationalTol);

        //********************** VERBOSE ***********************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                printMessage(0,"Module verbosity set to %i\n",verbosity);
            }
            else printMessage(0,"Could not find verbosity option in config file; using %i as default.\n",verbosity);

        //******************** SOURCE_MODE *********************
            if (rf.check("src_mode"))
            {
                if (rf.find("src_mode").asString() == "test" || rf.find("src_mode").asString() == "DH")// || rf.find("src_mode").asString() == "URDF")
                {
                    src_mode = rf.find("src_mode").asString();
                    printMessage(0,"src_mode set to %s\n",src_mode.c_str());
                }
                else printMessage(0,"src_mode option found but not allowed; using %s as default.\n",src_mode.c_str());
            }
            else printMessage(0,"Could not find src_mode option in the config file; using %s as default.\n",src_mode.c_str());

        //********************** DH_FILE ***********************
            if (src_mode == "DH")
            {        
                if (rf.check("DH_file"))
                {
                    DH_file = rf.find("DH_file").asString();
                    printMessage(0,"DH_file set to %s\n",DH_file.c_str());
                }
                else printMessage(0,"Could not find DH_file option in the config file; using %s as default.\n",DH_file.c_str());
            }

        //********************* URDF_FILE **********************
            if (src_mode == "URDF")
            {        
                if (rf.check("URDF_file"))
                {
                    URDF_file = rf.find("URDF_file").asString();
                    printMessage(0,"URDF_file set to %s\n",URDF_file.c_str());
                }
                else printMessage(0,"Could not find URDF_file option in the config file; using %s as default.\n",URDF_file.c_str());
            }

        //******************** OUTPUT_FILE *********************
            homePath = rf.getHomeContextPath().c_str();
            homePath = homePath+"/";
            outputFile = rf.check("outputFile", Value("output.ini")).asString().c_str();
            printMessage(0,"Storing file set to: %s\n", (homePath+outputFile).c_str());

        if(!initVariables())      return false;
        if(!configureInvKin())    return false;

        //********************* THREAD(s) **********************
            for (int i = 0; i < 1; i)
            {
                wsEvThreads.push_back(new workspaceEvThread(rate,verbosity,name,translationalTol,
                                                            orientationalTol,*chain,poss2Expl,
                                                            oris2Expl,(homePath+outputFile)));

                wsEvThreads.back()->start();
            }
            printMessage(0,"workspaceEvThreads have been istantiated...\n");
        return true;
    }

    /**
     * Initializes some variables, such as the workspace limits and the vectors that store the workspace
     * @return true/false if success/failure
     */
    bool initVariables()
    {
        //*********** WORKSPACE LIMITS ************
            Bottle &wl = rf.findGroup("WORKSPACE_LIMITS");

            if (!wl.isNull())
            {
                printMessage(0,"Found custom workspace limits.\n");
                Bottle *wlX = wl.find("limX").asList();
                Bottle *wlY = wl.find("limY").asList();
                Bottle *wlZ = wl.find("limZ").asList();

                wsLims(0,0) = wlX->get(0).asDouble();    wsLims(0,1) = wlX->get(1).asDouble();
                wsLims(1,0) = wlY->get(0).asDouble();    wsLims(1,1) = wlY->get(1).asDouble();
                wsLims(2,0) = wlZ->get(0).asDouble();    wsLims(2,1) = wlZ->get(1).asDouble();
            }
            else
            {
                wsLims(0,0) = -0.5;                wsLims(0,1) =  0.1;
                wsLims(1,0) = -0.6;                wsLims(1,1) =  0.6;
                wsLims(2,0) = -0.3;                wsLims(2,1) =  0.7;
            }
            printMessage(0,"Workspace Limits set to: \n%s\n",wsLims.toString(3,3).c_str());

        //******** POSITIONS 2 EXPLORE ************
            // Populate the vector of positions in order for it to be used later
            printMessage(0,"Populating the vectors..\n");
            Vector p(3,0.0);
            for (double i = wsLims(0,0); i <= wsLims(0,1); i=i+granP)
            {
                for (double j = wsLims(1,0); j <= wsLims(1,1); j=j+granP)
                {
                    for (double k = wsLims(2,0); k <= wsLims(2,1); k=k+granP)
                    {
                        p(0)=i;
                        p(1)=j;
                        p(2)=k;
                        printMessage(3,"poss2Expl: %s\n",p.toString(3,3).c_str());
                        poss2Expl.push_back(p);
                        reachability.push_back(0.0);
                    }
                }
            }
            printMessage(0,"Vectors have been populated!\n");

        //******** ORIENTATIONS 2 EXPLORE **********
            // Populate the vector of orientations in order for it to be used later on
            Vector zyz(3,0.0);
            int tick = 120;             // in degrees

            for (int i = 0; i < 360; i=i+tick)
            {
                for (int j = 0; j < 360; j=j+tick)
                {
                    for (int k = 0; k < 360; k=k+tick)
                    {
                        zyz(0)=CTRL_DEG2RAD*i;
                        zyz(1)=CTRL_DEG2RAD*j;
                        zyz(2)=CTRL_DEG2RAD*k;

                        oris2Expl.push_back(dcm2axis(euler2dcm(zyz)));
                        printMessage(3,"oris2Expl: %s\n",oris2Expl.back().toString(3,3).c_str());
                    }
                }
            }

        return true;
    }

    /**
     * Configures the chain according to the src_mode. It can be either a test chain (a classical iCubArm left),
     * a DH file, or an URDF file. After this, it instantiates a proper iKinIpOptMin solver.
     * @return true/false if success/failure
     */
    bool configureInvKin()
    {
        if (src_mode == "test")
        {
            arm   = new iCubArm("left");
            chain = arm->asChain();

            // Relase torso joints since we want to explore as much workspace as possible
            chain->releaseLink(0);
            chain->releaseLink(1);
            chain->releaseLink(2);
        }
        else if (src_mode == "DH")
        {
            string file = rf.findFile(DH_file.c_str());
            printMessage(0,"DH_file found: %s\n",file.c_str());

            Property linksOptions;
            linksOptions.fromConfigFile(file.c_str());
            limb=new iKinLimb(linksOptions);
            chain = limb->asChain();
        }
        else if (src_mode == "URDF")
        {
            return false;
        }
        printMessage(1,"The chain has been successfully instantiated.\n");

        return true;
    }

    bool updateModule()
    {
        // Periodically check if all the threads have finished:
        bool areJobsDone=1;

        for (int i = 0; i < wsEvThreads.size(); i++)
        {
            if (!wsEvThreads[i]->checkJobDone())
            {
                areJobsDone = areJobsDone && 0;
            }
        }

        if (areJobsDone)
        {
            stopModule();
        }
        return true;
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

    bool close()
    {   
        printMessage(0,"Stopping threads..\n");
        for (int i = 0; i < wsEvThreads.size(); i++)
        {
            wsEvThreads[i]->stop();
            delete wsEvThreads[i];
            wsEvThreads[i]=0;
        }

        if (arm)
        {
            delete arm;
            arm=0;
        }
        if (limb)
        {
            delete limb;
            limb=0;
        }

        return true;
    }

    double getPeriod()
    {
        return 0.5;
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
        cout<<endl<<"Options:"<<endl;
        cout<<"   --context          path:   where to find the called resource (default periPersonalSpace)."<<endl;
        cout<<"   --from             from:   the name of the .ini file (default workspaceEvaluator.ini)."<<endl;
        cout<<"   --name             name:   the name of the module (default workspaceEvaluator)."<<endl;
        cout<<"   --verbosity        int:    verbosity level (default 0)."<<endl;
        cout<<"   --rate             int:    the period used by the module. Default 500ms."<<endl;
        cout<<"   --granP            double: the spatial granularity of the workspace exploration. Default 1cm."<<endl;
        cout<<"   --translationalTol double: the translational tolerance used to assess if a point in the workspace has"<<endl;
        cout<<"                              been reached. Default 5e-3m"<<endl;
        cout<<"   --orientationalTol double: the orientational tolerance used to assess if a point in the workspace has"<<endl;
        cout<<"                              been reached. Default 20[deg]"<<endl;
        cout<<"   --src_mode         mode:   source to use finding the chain (either test, DH, or URDF; default test)."<<endl;
        cout<<"                              NOTE:"<<endl;
        cout<<"                              \'test\' creates a right iCubArm and tests the software on it;"<<endl;
        cout<<"                              \'DH\'   needs a proper DH.ini with the specification of the chain in DH convention;"<<endl;
        cout<<"                              \'URDF\' needs a proper URDF file."<<endl;
        cout<<"   --DH_file          string: DH.ini file to be used alongside the \'DH\' src_mode."<<endl;
        cout<<"   --URDF_file        string: URDF   file to be used alongside the \'URDF\' src_mode."<<endl;
        cout<<endl;
        return 0;
    }

    workspaceEvaluator workEv;
    return workEv.runModule(moduleRF);
}

// empty line to make gcc happy

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
    double granP;
    double translationalTol;
    double orientationalTol;

    Matrix sLims;

    iKinChain    *chain;
    iKinIpOptMin *slv;

    vector<Vector> poss2Expl;
    vector<double> reachability;

    Vector foo;

public:
    workspaceEvaluator()
    {
        foo.resize(3,0.0);
        isJobDone = 0;

        // These are the limits of the exploration of the workspace, defined as a 3x2 matrix
        // sLims = [minX maxX; minY maxY; minZ maxZ]
        sLims.resize(3,2);
        sLims(0,0) = -0.4;
        sLims(0,1) =  0.1;
        sLims(1,0) = -0.4;
        sLims(1,1) =  0.4;
        sLims(2,0) = -0.1;
        sLims(2,1) =  0.5;

        // Populate the vector of positions in order for it to be used later
        Vector p(3,0.0);
        for (double i = sLims(0,0); i < sLims(0,1); i=i+granP)
        {
            for (double j = sLims(1,0); j < sLims(1,1); j=j+granP)
            {
                for (double k = sLims(2,0); k < sLims(2,1); k=k+granP)
                {
                    p(0)=i;
                    p(1)=j;
                    p(2)=k;
                    poss2Expl.push_back(p);
                    reachability.push_back(0.0);
                }
            }
        }
    }

    bool configure(ResourceFinder &rf)
    {
        name        = "workspaceEvaluator"; // name
        verbosity   = 0;                    // verbosity
        rate        = 0.5;                  // rate
        src_mode    = "test";               // src_mode
        granP       = 0.01;                 // spatial granularity
        translationalTol = 5e-3;
        orientationalTol = 1e-8;

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
            if (rf.check("granP"))
            {
                granP = rf.find("granP").asDouble();
                printMessage(0,"Each joint will be divided into %g granP\n", granP);
            }
            else printMessage(0,"Could not find granP in the config file; using %g as default.\n",granP);

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

            configureInvKin();

        return true;
    }

    bool close()
    {   
        if (slv)
        {
            delete slv;
            slv = 0;
        }
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
            exploreWorkspace();
            saveWorkspace();
            isJobDone = 1;
        }
        else
        {
            printMessage(0,"Finished\n");
            close();
        }
        return true;
    }

    /**
     * Explores the workspace by iteratively span all the joints and store the position into a suitable variable.
     * @param  jnt the link from which the exploration starts (usually 0)
     * @return     true/false if success/failure
     */
    bool exploreWorkspace()
    {
        Vector pos2Expl(3,0.0);  // 3D position    to explore
        Vector ori2Expl(4,0.0);  // 4D orientation to explore (axis-angle)
        Vector pose2Expl(7,0.0); // 7D full pose   to explore

        Vector posObt(3,0.0);    // 3D position    that have been actually obtained
        Vector oriObt(4,0.0);    // 4D orientation that have been actually obtained
        Vector poseObt(7,0.0);   // 7D full pose   that have been actually obtained

        for (int i = 0; i < poss2Expl.size(); i++)
        {
            /* TODO put the code for spanning the orientation */

            pos2Expl = poss2Expl[i];
            pose2Expl.setSubvector(0,pos2Expl);
            pose2Expl.setSubvector(3,ori2Expl);

            Vector qhat = slv->solve(chain->getAng(),pose2Expl);
            // printMessage(0,"qhat")
            poseObt=chain->EndEffPose();
            posObt=poseObt.subVector(0,2);
            oriObt=poseObt.subVector(3,6);

            if (norm(pos2Expl-posObt)<translationalTol)// && norm(ori2Expl-oriObt)<orientationalTol)
            {
                printMessage(1,"Pose %s has been successfully reached!\n", pos2Expl.toString().c_str());
            }
            else
            {
                printMessage(2,"Pose %s has not been reached. norm(pos2Expl-posObt)=%g\n", pos2Expl.toString().c_str(), norm(pos2Expl-posObt));
            }
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
     * a DH file, or an URDF file. After this, it instantiates a proper iKinIpOptMin solver.
     * @return true/false if success/failure
     */
    bool configureInvKin()
    {
        if (src_mode == "test")
        {
            printf("0\n");
            iCubArm *arm = new iCubArm("left");
            chain = arm->asChain();

            // Relase torso joints since we want to explore as much workspace as possible
            chain->releaseLink(0);
            chain->releaseLink(1);
            chain->releaseLink(2);

            // TODO: handle the deletion of the icubArm!
            // delete arm;
            // arm = 0;
        }
        else if (src_mode == "DH")
        {
            return false;
        }
        else if (src_mode == "URDF")
        {
            return false;
        }
        printf("asofji\n");
        // instantiate a IPOPT solver for inverse kinematic
        // for both translational and rotational part
        slv = new iKinIpOptMin(*chain,IKINCTRL_POSE_FULL,1e-3,100);  

        // we have a dedicated tolerance for the translational part
        // which is by default equal to 1e-6;
        // note that the tolerance is applied to the squared norm
        slv->setTranslationalTol(1e-8);
        printf("asdac\n");


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
        cout << "   --context   path:   where to find the called resource (default periPersonalSpace)." << endl;
        cout << "   --from      from:   the name of the .ini file (default workspaceEvaluator.ini)." << endl;
        cout << "   --name      name:   the name of the module (default workspaceEvaluator)." << endl;
        cout << "   --verbosity int:    verbosity level (default 0)." << endl;
        cout << "   --rate      int:    the period used by the module. Default 500ms." << endl;
        cout << "   --granP     double: the spatial granularity of the workspace exploration. Default 1cm." << endl;
        cout << "   --src_mode  mode:   source to use finding the chain (either test, DH, or URDF; default test)." << endl;
        cout << endl;
        return 0;
    }

    workspaceEvaluator workEv;
    return workEv.runModule(moduleRF);
}

// empty line to make gcc happy

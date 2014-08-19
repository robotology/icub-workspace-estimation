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

#include <iostream>
#include <string>
#include <vector>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;

using namespace std;

class workspaceEvaluator: public RFModule 
{
private:

    RpcClient        rpcClnt;
    RpcServer        rpcSrvr;

    string name;
    int verbosity;
    int rate;

public:
    workspaceEvaluator()
    {
    }

    bool configure(ResourceFinder &rf)
    {
        name      = "workspaceEvaluator"; // name
        verbosity = 0;                    // verbosity
        rate      = 0.05;                 // rate

        //******************************************************
        //********************** CONFIGS ***********************

        //********************* NAME *******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                cout << "Module name set to "<<name<<endl;  
            }
            else cout << "Module name set to default, i.e. " << name << endl;
            setName(name.c_str());


        //******************** RATE ********************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt()/1000;
                cout << "workspaceEvaluator working at " << rate << " s.\n";
            }
            else cout << "Could not find rate in the config file; using "
                      << rate << " s as default.\n";

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                cout << "workspaceEvaluator verbosity set to " << verbosity << endl;
            }
            else cout << "Could not find verbosity option in " <<
                         "config file; using "<< verbosity <<" as default.\n";

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
        return true;
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
        cout << "   --rate      int:   the period used by the module. Default 50ms." << endl;
        cout << endl;
        return 0;
    }

    workspaceEvaluator workEv;
    return workEv.runModule(moduleRF);
}

// empty line to make gcc happy

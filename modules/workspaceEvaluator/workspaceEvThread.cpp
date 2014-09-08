#include "workspaceEvThread.h"

workspaceEvThread::workspaceEvThread(int _rate, int _v, string _n, double _tT, const iKinChain &_c,
                                     const vector<Vector> &_p2E, string _oF, string _sM,
                                     string _eVM, string _eXM, double _gP, int _rJ) :
                                     RateThread(_rate), verbosity(_v), name(_n), XYZTol(_tT),
                                     outputFile(_oF), rate(_rate), src_mode(_sM), eval_mode(_eVM),
                                     expl_mode(_eXM), granP(_gP), resolJ(_rJ)
{
    explVec = _p2E;
    chain   = _c;
    printMessage(4,"normalConstructor name %s chainDOF %i\n",name.c_str(),chain.getDOF());
}

workspaceEvThread::workspaceEvThread(const workspaceEvThread &_wET) :
                                     RateThread(_wET.getRate()), rate(_wET.getRate()),
                                     verbosity(_wET.getVerbosity()), name(_wET.getName()),
                                     XYZTol(_wET.getXYZTol()), outputFile(_wET.getOutputFile()),
                                     src_mode(_wET.getSrcMode()), eval_mode(_wET.getEvalMode()),
                                     expl_mode(_wET.getExplMode()), granP(_wET.getGranP()),
                                     resolJ(_wET.getResolJ())
{
    chain   = _wET.getChain();
    explVec = _wET.getExplVec();

    printMessage(4,"copyConstructor name %s chainDOF %i\n",name.c_str(),chain.getDOF());
}

bool workspaceEvThread::threadInit()
{
    cnt       = 0;
    step      = 0;
    isJobDone = 0;
    timeStart = 0;
    timeEnd   = 0;

    slv = new iKinIpOptMin(chain,IKINCTRL_POSE_XYZ,1e-8,200);

    for (int i = 0; i < explVec.size(); i++)
    {
        reachability.push_back(-1.0);
    }

    printMessage(0,"threadInit name %s #explVec %i\n",name.c_str(),explVec.size());

    return true;
}

void workspaceEvThread::run()
{
    switch (step)
    {
        case 0:
            timeStart = yarp::os::Time::now();
            printMessage(0,"STARTING EXPLORATION...\n");
            step++;
            break;
        case 1:
            exploreWorkSpace();
            break;
        case 2:
            printMessage(0,"FINISHED.\n");
            timeEnd = yarp::os::Time::now();
            printMessage(0,"Elapsed time: %g\n",timeEnd-timeStart);
            isJobDone=1;
            step++;
            break;
        default:
            Time::delay(1);
            break;
    }

}

bool workspaceEvThread::exploreWorkSpace()
{
    if (expl_mode == "operationalSpace")
    {
        return exploreOperationalSpace();
    }
    else if (expl_mode == "jointSpace")
    {
        return exploreJointSpace();
    }
    return false;
}

bool workspaceEvThread::exploreJointSpace(const int &jnt)
{
    double min = (chain)[jnt].getMin();
    double max = (chain)[jnt].getMax();
    double increment = (max-min)/(resolJ-1);

    // The spaces are there in order to make some order into the printouts (otherwise its a mess)
    string spaces="";
    for (int i = 0; i < jnt; i++)
    {
        spaces += " ";
    }
    printMessage(jnt,(spaces+"Analyzing link #%i : min %g\tmax %g\tincrement %g\n").c_str(),jnt,min,max,increment);
    chain.setAng(jnt,min);

    while (chain.getAng(jnt) <= max+1e-5)
    {
        if (jnt+1 < chain.getN())
        {
            exploreJointSpace(jnt+1);
        }
        else
        {
            cnt++;
            explVec.push_back(chain.getH().getCol(3).subVector(0,2));
            reachability.push_back(computeManipulability());
        }

        printMessage(jnt+1,(spaces+"Link #%i ang set to %g\n").c_str(),jnt,chain.getAng(jnt));  
        chain.setAng(jnt,chain.getAng(jnt)+increment);
    }

    if (cnt==pow(resolJ,chain.getN()))
    {
        step=2;
    }
    return true;
}

bool workspaceEvThread::exploreOperationalSpace()
{
    Vector pos2Expl(3,0.0);  // 3D position    to explore
    Vector posObt(3,0.0);    // 3D position    that have been actually obtained
    Vector poseObt(7,0.0);   // 7D full pose   that have been actually obtained

    pos2Expl = explVec[cnt];

    printMessage(2,"Advancement: %g\tExploring %s starting from %s..\n",pos2Expl.toString(3,3).c_str(),
                    getAdvancement(),chain.getAng().toString(3,3).c_str());
    Vector qhat = slv->solve(chain.getAng(),pos2Expl);

    poseObt=chain.EndEffPose();
    posObt =poseObt.subVector(0,2);
    
    if (norm(pos2Expl-posObt)<XYZTol)
    {
        reachability[cnt] = computeManipulability();
        printMessage(1,"%s\thas been successfully reached! Manipulability: %g\n",
                        pos2Expl.toString(3,3).c_str(),reachability[cnt]);
    }
    else
    {
        printMessage(3,"%s\thas not been reached. norm(poss)=%g\tposObt=%s\n",
                    pos2Expl.toString(3,3).c_str(), norm(pos2Expl-posObt),
                    posObt.toString(3,3).c_str());
    }

    cnt++;
    if (cnt==explVec.size())
    {
        step++;
    }

    // Print something every 5% of advancement
    if (int(getAdvancement()*1000)%5000 == 0)
    {
        printMessage(0,"Advancement: %g\n\n",getAdvancement());
    }

    return true;
}

double workspaceEvThread::computeManipulability()
{
    /*
     * Manipulability is defined by sqrt(|J*J^T|), where J is the 
     * Analytical (aka Task) Jacobian matrix
     */
    
    // Let's take only the positional components of the jacobian
    // Matrix J_a   = chain.AnaJacobian();
    Matrix J_a   = chain.GeoJacobian();

    return sqrt(det(J_a * J_a.transposed()));
}

bool workspaceEvThread::saveWorkspace()
{
    printMessage(0,"Saving exploration to %s\n",outputFile.c_str());

    int reached = 0;
    for (int i = 0; i < reachability.size(); i++)
    {
        if (reachability[i]>= 0.0)
        {
            reached += 1;
        }
    }
    printMessage(1,"Number of poses explored: %i\tNumber of poses reached: %i\n",
                    cnt,reached);
    Bottle data;
    ofstream myfile;
    myfile.open(outputFile.c_str(),ios::trunc);

    // Print some info on the test that has been done:
    myfile << "########################################################" << endl;
    myfile << "# src_mode  " << src_mode  << endl;
    myfile << "# eval_mode " << eval_mode << endl;
    myfile << "# expl_mode " << expl_mode << endl;
    myfile << "########################################################" << endl;

    if (myfile.is_open())
    {
        for (int i = 0; i < explVec.size(); i++)
        {
            data.clear();
            // Round the doubles to the cm before saving:
            for (int j = 0; j < 3; j++)
            {
                double tmp  = explVec[i][j];
                double tmp2 = tmp*100.0;

                if (tmp-floor(tmp)>=0.5)
                {
                    data.addDouble(ceil(tmp2)/100.0);
                }
                else
                {
                    data.addDouble(floor(tmp2)/100.0);
                }
            }

            data.addDouble(reachability[i]);
            myfile << data.toString() << endl;
        }
    }
    myfile.close();
    printMessage(1,"File saved.\n");

    return true;
}

int workspaceEvThread::printMessage(const int l, const char *f, ...) const
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

void workspaceEvThread::threadRelease()
{
    saveWorkspace();

    if (slv)
    {
        delete slv;
        slv = 0;
    }
}

// empty line to make gcc happy

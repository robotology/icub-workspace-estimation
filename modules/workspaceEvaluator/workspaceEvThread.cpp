#include "workspaceEvThread.h"

workspaceEvThread::workspaceEvThread(int _rate, int _v, string _n, double _tT, double _oT,
                                     const iKinChain &_c, const vector<Vector> &_p2E,
                                     const vector<Vector> &_o2E, string _oF) :
                                     RateThread(_rate), verbosity(_v), name(_n),
                                     translationalTol(_tT), orientationalTol(_oT),
                                     outputFile(_oF), rate(_rate)
{
    printMessage(0,"norm name %s chainDOF %i\n\n",name.c_str(),chain.getDOF());
    poss2Expl = _p2E;
    oris2Expl = _o2E;
    chain = _c;
}

workspaceEvThread::workspaceEvThread(const workspaceEvThread &_wET):
                                     RateThread(_wET.getRate()), rate(_wET.getRate())
{
    verbosity = _wET.getVerbosity();
    name      = _wET.getName();
    chain     = _wET.getChain();

    translationalTol = _wET.getTranslationalTol();
    orientationalTol = _wET.getOrientationalTol();

    outputFile = _wET.getOutputFile();

    poss2Expl = _wET.getPoss2Expl();
    oris2Expl = _wET.getOris2Expl();

    printMessage(0,"copy name %s chainDOF %i\n\n",name.c_str(),chain.getDOF());
}

bool workspaceEvThread::threadInit()
{
    cnt       = 0;
    step      = 0;
    isJobDone = 0;

    slv = new iKinIpOptMin(chain,IKINCTRL_POSE_FULL,1e-3,100);  
    slv->setTranslationalTol(1e-8);

    for (int i = 0; i < poss2Expl.size(); i++)
    {
        reachability.push_back(0);
    }

    printMessage(0,"init name %s chainDOF %i\n\n",name.c_str(),chain.getDOF());

    return true;
}

void workspaceEvThread::run()
{
    switch (step)
    {
        case 0:
            printMessage(0,"STARTING EXPLORATION...\n");
            step++;
            break;
        case 1:
            if (!isJobDone)
            {       
                exploreWorkspace();
            }
            else
            {
                saveWorkspace();
                step++;
            }
            break;
        default:
            printMessage(0,"FINISHED.\n");
            Time::delay(1);
            break;
    }

}

bool workspaceEvThread::exploreWorkspace()
{
    Vector pos2Expl(3,0.0);  // 3D position    to explore
    Vector ori2Expl(4,0.0);  // 4D orientation to explore (axis-angle)
    Vector eul2Expl(3,0.0);  // 3D orientation to explore (euler notation)
    Vector pose2Expl(7,0.0); // 7D full pose   to explore

    Vector posObt(3,0.0);    // 3D position    that have been actually obtained
    Vector oriObt(4,0.0);    // 4D orientation that have been actually obtained
    Vector eulObt(3,0.0);    // 3D orientation that have been actually obtained
    Vector poseObt(7,0.0);   // 7D full pose   that have been actually obtained

    pos2Expl = poss2Expl[cnt];
    pose2Expl.setSubvector(0,pos2Expl);

    for (int j = 0; j < oris2Expl.size(); j++)
    {
        ori2Expl = oris2Expl[j];
        eul2Expl=CTRL_RAD2DEG*dcm2euler(axis2dcm(ori2Expl));
        pose2Expl.setSubvector(3,ori2Expl);

        Vector qhat = slv->solve(chain.getAng(),pose2Expl);
        poseObt=chain.EndEffPose();
        posObt=poseObt.subVector(0,2);
        oriObt=poseObt.subVector(3,6);
        eulObt=CTRL_RAD2DEG*dcm2euler(axis2dcm(oriObt));

        if (norm(pos2Expl-posObt)<translationalTol && norm(eul2Expl-eulObt)<orientationalTol)
        {
            reachability[cnt] += 1.0;
            printMessage(1,"%s\t%s has been successfully reached!\n",
                        pos2Expl.toString(3,3).c_str(), eul2Expl.toString(3,3).c_str());
        }
        else
        {
            printMessage(2,"%s\t%s has not been reached. norm(poss)=%g\t norm(euls)=%g\n",
                        pos2Expl.toString(3,3).c_str(), eul2Expl.toString(3,3).c_str(),
                        norm(pos2Expl-posObt), norm(eul2Expl-eulObt));
            printMessage(4,"%s\t%s\n", posObt.toString(3,3).c_str(), eulObt.toString(3,3).c_str());
        }
    }

    cnt++;
    if (cnt==poss2Expl.size())
    {
        isJobDone=1;
    }

    return true;
}

bool workspaceEvThread::saveWorkspace()
{
    printMessage(0,"SAVING EXPLORATION to %s\n",outputFile.c_str());

    int reached = 0;
    for (int i = 0; i < reachability.size(); i++)
    {
        reached += reachability[i];
    }
    printMessage(1,"Number of poses explored: %i\tNumber of poses reached: %i\n",cnt*oris2Expl.size(),reached);
    Bottle data;
    ofstream myfile;
    myfile.open(outputFile.c_str(),ios::trunc);

    if (myfile.is_open())
    {
        for (int i = 0; i < poss2Expl.size(); i++)
        {
            data.clear();
            data.addDouble(poss2Expl[i](0));
            data.addDouble(poss2Expl[i](1));
            data.addDouble(poss2Expl[i](2));
            data.addDouble(reachability[i]);
            myfile << data.toString() << endl;
        }
    }
    myfile.close();

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

#include "workspaceEvThread.h"

workspaceEvThread::workspaceEvThread(int _rate, int _v, string _n, double _tT,
                                     const iKinChain &_c, const vector<Vector> &_p2E,
                                     string _oF) :
                                     RateThread(_rate), verbosity(_v), name(_n),
                                     translationalTol(_tT), outputFile(_oF), rate(_rate)
{
    poss2Expl = _p2E;
    chain = _c;
    printMessage(0,"normalConstructor name %s chainDOF %i\n",name.c_str(),chain.getDOF());
}

workspaceEvThread::workspaceEvThread(const workspaceEvThread &_wET):
                                     RateThread(_wET.getRate()), rate(_wET.getRate()),
                                     verbosity(_wET.getVerbosity()),name(_wET.getName()),
                                     translationalTol(_wET.getTranslationalTol()),
                                     outputFile(_wET.getOutputFile())
{
    chain     = _wET.getChain();
    poss2Expl = _wET.getPoss2Expl();

    printMessage(0,"copyConstructor name %s chainDOF %i\n",name.c_str(),chain.getDOF());
}

bool workspaceEvThread::threadInit()
{
    cnt       = 0;
    step      = 0;
    isJobDone = 0;

    slv = new iKinIpOptMin(chain,IKINCTRL_POSE_XYZ,1e-8,200);

    for (int i = 0; i < poss2Expl.size(); i++)
    {
        reachability.push_back(0.0);
    }

    printMessage(0,"threadInit name %s chainDOF %i\n",name.c_str(),chain.getDOF());

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
    Vector posObt(3,0.0);    // 3D position    that have been actually obtained
    Vector poseObt(7,0.0);   // 7D full pose   that have been actually obtained

    pos2Expl = poss2Expl[cnt];

    printMessage(2,"Exploring %s..\n",pos2Expl.toString(3,3).c_str());
    Vector qhat = slv->solve(chain.getAng(),pos2Expl);
    poseObt=chain.EndEffPose();
    posObt=poseObt.subVector(0,2);
    
    if (norm(pos2Expl-posObt)<translationalTol)
    {
        reachability[cnt] = computeManipulability();
        printMessage(1,"%s\thas been successfully reached! Manipulability: %g\n",
                        pos2Expl.toString(3,3).c_str(),reachability[cnt]);
    }
    else
    {
        printMessage(2,"%s\thas not been reached. norm(poss)=%g\tposObt=%s\n",
                    pos2Expl.toString(3,3).c_str(), norm(pos2Expl-posObt),
                    posObt.toString(3,3).c_str());
    }

    cnt++;
    if (cnt==poss2Expl.size())
    {
        isJobDone=1;
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
    Matrix J_a   = chain.AnaJacobian().submatrix(0,2,0,chain.getDOF());

    return sqrt(det(J_a * J_a.transposed()));
}

bool workspaceEvThread::saveWorkspace()
{
    printMessage(0,"SAVING EXPLORATION to %s\n",outputFile.c_str());

    int reached = 0;
    for (int i = 0; i < reachability.size(); i++)
    {
        reached += bool(reachability[i]);
    }
    printMessage(1,"Number of poses explored: %i\tNumber of poses reached: %i\n",
                    cnt*oris2Expl.size(),reached);
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

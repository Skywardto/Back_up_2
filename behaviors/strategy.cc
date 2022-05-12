#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"

#include<algorithm>//quick sort
#include<cmath>
using namespace std;
#define PASS_CONS 100.00
#define SHOOT_CONS 100.00
#define TEAMMATE_FIT_DIS 5.00
#define BALL_FIT_DIS 5.00

void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) 
{
    beamX = -HALF_FIELD_X + worldModel->getUNum();
    beamY = 0;
    beamAngle = 0;
}

struct NumVec
{
   VecPosition Vec;
   int Num;
   int serial;
}NumVecTemp,UsClosest,OppClosest,NUMPASS;

struct VecInd
{
    VecPosition Vec;
    double Ind;
}little_det_tmp[8];

VecPosition Ball_closest_member(NumVec tmp, double d, int t);
bool Is_pass_open(VecPosition t1,VecPosition t2);
void Goalie();
void Swap_make_increasing(VecPosition& a,VecPosition& b);
void Distri_one_area(NumVec &NumVecTemp, int t);
void model_select(VecPosition ball);
void Conpensate(VecInd(&target)[12],int ahead);
void Attack_model(VecPosition ball);
double Evaluate_pass_index(VecPosition a,int serial);
double Evaluate_line_warn_index(VecPosition t1, VecPosition t2);
void Conpensate(VecInd(&target)[12],int ahead);
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


struct NumInd
{
    int Num;
    double Ind;
};

bool cmp(NumInd a,NumInd b)
{
    return a.Ind <b.Ind;
}

bool cmp1(VecInd a,VecInd b)
{
    return a.Ind <b.Ind;
}

double us_closest_distance_toball=1000.00;
double opp_closest_distance_toball=1000.00;
int ORDER=0;
double SHOOT_TAR_INDEX=0;


struct Task
{
    int Num;
    VecPosition tar;
    int kind;
}task[11]={};


struct area
{
    vector<NumVec> us;
    vector<NumVec> opp;
    
}area[5];

void model_select(VecPosition ball)
{
    if(ball.getX()<-10 )
    Attack_model(ball);
    // else if(ball.getX()<5)
    // Equili_model();
    else
    Attack_model(ball);
    // Defense_model();
}


VecPosition Ball_closest_member(NumVec tmp,double d,int t)
{
    switch (t)
    {
    case 1:
        if(d<us_closest_distance_toball)
        {
            UsClosest=tmp;     
            us_closest_distance_toball=d;       
        }

        break;
    
    case 2:
        if(d<opp_closest_distance_toball)
        {
            OppClosest=tmp;    
            opp_closest_distance_toball=d;        
        }

        break;
    }
}


void Swap_make_increasing(VecPosition& a,VecPosition& b)
{
    if(a.getX()<b.getX())
    {}
    else
    {
        VecPosition c=a;
        a=b;
        b=c;
    }
}

bool Is_pass_open(VecPosition t1,VecPosition t2)//the method will have a problem that bevel part's evluation is inaccurate
{
    VecPosition a=t1;
    VecPosition b=t2;
    Swap_make_increasing(a,b);
    double slope=( a.getY()-b.getY() )/( a.getX()-b.getX() );
    double det=1/( cos( atan(slope) ) );

    for(int i=0;i<5;i++)
    {
        for(int j=0;j<area[i].opp.size();j++)
        {
            if(area[i].opp[j].Vec.getX()>a.getX()&&area[i].opp[j].Vec.getX()<b.getX())
            {
                if( (area[i].opp[j].Vec.getY()>( slope*area[i].opp[j].Vec.getX()-det ) )&&( area[i].opp[j].Vec.getY()<( slope*area[i].opp[j].Vec.getX()+det )) )
                {
                    return false;
                }  
            }
             
        }
        for(int k=0;k<area[i].us.size();k++)
        {
            if(area[i].us[k].Vec.getX()>a.getX()&&area[i].us[k].Vec.getX()<b.getX())
            if(area[i].us[k].Vec.getY()>( slope*area[i].us[k].Vec.getX()-det )&&( area[i].us[k].Vec.getY()<( slope*area[i].us[k].Vec.getX()+det )) )
            {
                return false;
            }  
        }
    }
    return true;

}

// void Evaluate_pass_index(NumVec tmp)
// {
//     int k1=1,k2=1;
//     Pass_avai a;
//     a.avai_us=tmp;
//     a.pass_index=k1*
//     pass_que.push_back()
// }
// void Strain_act(VecPosition a)
// {
//     Whether_pass_ball(a);
//     Whether_shoot(a);
// }

double Evaluate_pass_index(VecPosition a,int serial)
{
    double result=0;
    int k1=1,k2=1,k3=100;//constent


    for(int i=0;i<area[serial].opp.size();i++)
    {
       result+=k1* a.getDistanceTo(area[serial].opp[i].Vec);
    }

    // for(int i=0;i<area[serial].us.size();i++)
    // {
    //    result+=k2* abs(a.getDistanceTo(area[serial].us[i].Vec)-TEAMMATE_FIT_DIS);
    // }

    if(serial>0)
    {
         for(int i=0;i<area[serial-1].opp.size();i++)
        {
        result+=k1* a.getDistanceTo(area[serial-1].opp[i].Vec);
        }

        // for(int i=0;i<area[serial-1].us.size();i++)
        // {
        // result+=k2* abs(a.getDistanceTo(area[serial-1].us[i].Vec)-TEAMMATE_FIT_DIS);
        // }

    }

    if(serial<4)
    {
         for(int i=0;i<area[serial+1].opp.size();i++)
        {
        result+=k1* a.getDistanceTo(area[serial+1].opp[i].Vec);
        }

        // for(int i=0;i<area[serial+1].us.size();i++)
        // {
        // result+=k2* abs(a.getDistanceTo(area[serial+1].us[i].Vec)-TEAMMATE_FIT_DIS);
        // }

    }
            //search three nearby areas

    if(15-abs(a.getX())<1.5||10-abs(a.getY()<1.5))
    {
        result+=k3;
    }       //boundary warnning

}

bool Whether_pass_ball(NumVec usclosest)
{
    if(Evaluate_pass_index(usclosest.Vec,usclosest.serial)> PASS_CONS )
    {
        return true;
    }
    else
    return false;
}





NumVec Pass_who(VecPosition a,int serial)
{
    NumVec T;
    double min=1000;
    for(int i=0;i<area[serial].us.size();i++)
    {
        if(Is_pass_open(a,area[serial].us[i].Vec))
        {
            if(Evaluate_pass_index(area[serial].us[i].Vec,serial)<min)
            {
                min=Evaluate_pass_index(area[serial].us[i].Vec,serial);
                T.Num=area[serial].us[i].Num;
                T.Vec=area[serial].us[i].Vec;
                T.serial=serial;
            }
        }
    }

    if(serial>0)
    {
         for(int i=0;i<area[serial-1].us.size();i++)
        {
            if(Is_pass_open(a,area[serial-1].us[i].Vec))
            {
                if(Evaluate_pass_index(area[serial-1].us[i].Vec,serial-1)<min)
                {
                    min=Evaluate_pass_index(area[serial-1].us[i].Vec,serial-1);
                    T.Num=area[serial-1].us[i].Num;
                    T.Vec=area[serial-1].us[i].Vec;
                    T.serial=serial-1;
                }
            }

        }

    }

    if(serial<4)
    {
         for(int i=0;i<area[serial+1].opp.size();i++)
        {
            if(Is_pass_open(a,area[serial+1].us[i].Vec))
            {
                if(Evaluate_pass_index(area[serial+1].us[i].Vec,serial+1)<min)
                {
                    min=Evaluate_pass_index(area[serial+1].us[i].Vec,serial+1);
                    T.Num=area[serial+1].us[i].Num;
                    T.Vec=area[serial+1].us[i].Vec;
                    T.serial=serial+1;
                }
            }

        }

    }
    return T;
}


void Goalie(VecPosition ball)
{
    task[ORDER].Num=WO_TEAMMATE1;
    if(1)
    {
        task[ORDER].kind=1;        
    }
    else
    {
        task[ORDER].kind=1;//not complete
    }

    double y=-14.4*(OppClosest.Vec.getY()-ball.getY())/(OppClosest.Vec.getX()-ball.getX())+OppClosest.Vec.getY();
    if(y>-1.05&&y<1.05)
    {
        task[ORDER].tar=VecPosition(-14.4,y,0);
    }
    else
    {
        task[ORDER].tar=VecPosition(-14.4,0,0);
    }
 
    ++ORDER;
}

void Grab_position(VecPosition ball)
{
    task[ORDER].kind=1;
    task[ORDER].Num=UsClosest.Num;
    task[ORDER].tar=ball;
    ORDER++;

}

bool Is_hold()
{
    if(us_closest_distance_toball<1.5&&opp_closest_distance_toball>2)
    return true;
    else
    return false;
}

bool Is_shoot_open(VecPosition a,VecPosition b)//the method will have a problem that bevel part's evluation is inaccurate
{
    Swap_make_increasing(a,b);
    double slope=( a.getY()-b.getY() )/( a.getX()-b.getX() );
    double det=0.4/( cos( atan(slope) ) );//width

    for(int i=0;i<5;i++)
    {
        for(int j=0;j<area[i].opp.size();j++)
        {
            if(area[i].opp[j].Vec.getX()>a.getX()&&area[i].opp[j].Vec.getX()<b.getX())
            {
                if( (area[i].opp[j].Vec.getY()>( slope*area[i].opp[j].Vec.getX()-det ) )&&( area[i].opp[j].Vec.getY()<( slope*area[i].opp[j].Vec.getX()+det )) )
                {
                    return false;
                }  
            }
             
        }
        for(int k=0;k<area[i].us.size();k++)
        {
            if(area[i].us[k].Vec.getX()>a.getX()&&area[i].us[k].Vec.getX()<b.getX())
            if(area[i].us[k].Vec.getY()>( slope*area[i].us[k].Vec.getX()-det )&&( area[i].us[k].Vec.getY()<( slope*area[i].us[k].Vec.getX()+det )) )
            {
                return false;
            }  
        }
    }
    return true;

}

double Evaluate_shoot_index(VecPosition a,int serial)//not complete!!!!
{
    SHOOT_TAR_INDEX=0;//ini
    double slope1=-1,slope2=1,b1=-16.05,b2=16.05;

    double result=0;
    int k1=1,k2=1,k3=10;//constent

    if( a.getY()>(slope1*a.getX()+b1)&&a.getY()<(slope2*a.getX()+b2) )
    {
        result+=30;
    }
    
    for(int d=-0.7875;d<1.05;d+=0.525)
    {
        VecPosition t=(15,d,0);
        if(Is_shoot_open(a,t))
        {
            SHOOT_TAR_INDEX=d;
            return 110;//not complete!!!!I don't have time to make the decision best
        }

    }

}

bool Whether_shoot(NumVec a)
{
    if(a.Vec.getX()>7.5)
    {
        if(Evaluate_shoot_index(a.Vec,a.serial)>SHOOT_CONS)
        {
            return true;
        }
        
    }
    else
    {
        return false;
    }
}

VecPosition Shoot_where(VecPosition a,int serial)
{
    // VecPosition tar0=VecPosition(15,0,0);
    return VecPosition(15,SHOOT_TAR_INDEX,0);
}
VecInd Evaluate_point_warn_index(VecPosition p)
{
    double result=0;
    VecInd t;
    t.Vec=p;
    double r2=9;
    double cons1=1,cons2=0.7,cons3=200;

    for(int i=0;i<5;++i)
    {
        for(int j=0;j<area[i].opp.size();++j)
        {
            if(pow(area[i].opp[j].Vec.getY()-p.getY(),2)+pow(area[i].opp[j].Vec.getX()-p.getX(),2)<r2)
            {
                result+=cons1*p.getDistanceTo(area[i].opp[j].Vec);
            }
        }
        for(int j=0;j<area[i].us.size();++j)
        {
            if(pow(area[i].us[j].Vec.getY()-p.getY(),2)+pow(area[i].us[j].Vec.getX()-p.getX(),2)<r2)
            {
                result+=cons2*p.getDistanceTo(area[i].us[j].Vec);
            }
        }
    }
    
    if(15-abs(p.getX())<1.5||10-abs(p.getY()<1.5))
    {
        result+=cons3;
    }       //boundary warnning
    t.Ind=result;
    return t;
}
VecInd Find_min(VecInd tmp[8])
{
    int minNum=0;
    double minInd=tmp[0].Ind;
    for(int i=1;i<8;++i)
    {
        if(tmp[i].Ind<minInd)
        {
            minNum=i;
            minInd=tmp[i].Ind;
        }
    }
    return tmp[minNum];
}

int Find_min(double line[12])
{
    int minNum=0;
    double index=line[0];
    for(int i=0;i<12;++i)
    {
        if(line[i]<index)
        {
            index=line[i];
            minNum=i;
        }
    }
    return minNum;
}
void Attack_position()
{
    double detBx[12]={-1, -0.866, -0.5, 0, 0.5, 0.866, 1, 0.866, 0.5, 0, -0.5, -0.866};
    double detBy[12]={0, 0.5, 0.866, 1, 0.866, 0.5, 0, -0.5, -0.866, -1, -0.866, -0.5};
    double detLx[8]={-0.5, 0, 0.5, 0.5, 0.5, 0, -0.5, -0.5};
    double detLy[8]={0.5, 0.5, 0.5, 0, -0.5, -0.5, -0.5, 0};//moving list

    VecInd target_every[12];
    double line_index[12];
    task[ORDER].kind=1;

    NumVec core_;
    if(NUMPASS.Num==-1)
    {
        core_=UsClosest;
        task[ORDER].Num=UsClosest.Num;
    }
    else
    {
        core_=NUMPASS;
        task[ORDER].Num=NUMPASS.Num;
    }

    for(int i=0;i<12;i++)
    {
        for(int j=0;j<8;j++)
        {
            VecPosition tmp=VecPosition(core_.Vec.getX()+detBx[i]+detLx[j],core_.Vec.getX()+detBy[i]+detLy[j],0);
            little_det_tmp[j]=Evaluate_point_warn_index(tmp);
        }
        target_every[i]=Find_min(little_det_tmp);
        line_index[i]=Evaluate_line_warn_index(core_.Vec,target_every[i].Vec);
    }
    int ahead_Num=Find_min(line_index);

    task[ORDER].tar=target_every[ahead_Num].Vec;
        ++ORDER;//Until got target ,add one to ORDER 
    
    Conpensate(target_every,ahead_Num);

    sort(target_every,target_every+12,cmp1);
    for(int i=ORDER;i<11;++i)
    {
        task[i].kind=1;
        task[i].tar=target_every[i].Vec;
    }

    void Distri_task();
}

void Attack_model(VecPosition ball)
{
    NUMPASS.Num=-1;
    Goalie(ball);
    if(Is_hold())
    {
        if(Whether_pass_ball(UsClosest))
        {
            task[ORDER].kind=2;
            task[ORDER].Num=UsClosest.Num;
            NUMPASS=Pass_who(UsClosest.Vec,UsClosest.serial);
            task[ORDER].tar=NUMPASS.Vec;
            ++ORDER;

        }
        else if(Whether_shoot(UsClosest))
        {
            task[ORDER].kind=3;
            task[ORDER].Num=UsClosest.Num;
            task[ORDER].tar=Shoot_where(UsClosest.Vec,UsClosest.serial);
            ++ORDER;
        }
    }
    else
    {
        Grab_position(ball);
    }
    Attack_position();
}




void Conpensate(VecInd(&target)[12],int ahead)
{
    target[0].Ind+=0;
    target[1].Ind+=-10;
    target[2].Ind+=-12;
    target[3].Ind+=-15;
    target[4].Ind+=-12;
    target[5].Ind+=-10;
    target[6].Ind+=0;
    target[7].Ind+=5;
    target[8].Ind+=10;
    target[9].Ind+=15;
    target[10].Ind+=10;
    target[11].Ind+=5;

    target[(ahead+3+12)%12].Ind-=20;
    target[(ahead-3+12)%12].Ind-=20;
    target[(ahead+1+12)%12].Ind-=15;
    target[(ahead-1+12)%12].Ind-=15;//add weight to make it go ahead and go to right position

}


bool Is_contain(int num)
{
    for(int i=0;i<ORDER+1;++i)
    {
        if(num==task[i].Num)
        {
            return true;
        }
    }
    return false;
}

void Distri_task()
{
    for(;ORDER<11;++ORDER)//count by 1
    {
        NumInd d[11];
        int k=0;
        for(int i=0;i<5;++i)
        {
            for(int j=0;j<area[i].us.size();++j)
            {
                d[k].Ind=area[i].us[j].Vec.getDistanceTo(task[ORDER].tar);
                d[k].Num=area[i].us[j].Num;
                k++;
            }
        }
        sort(d,d+11,cmp);
        for(int n=0;n<11;++n)
        {
            if(Is_contain(d[n].Num))
            {
                continue;
            }
            else
            {
                task[ORDER].Num=d[n].Num;
                break;
            }
        }
    }

}

double Evaluate_line_warn_index(VecPosition t1,VecPosition t2)//I cheater here and up here
{
    VecPosition a=t1;
    VecPosition b=t2;
    Swap_make_increasing(a,b);
    double slope=( a.getY()-b.getY() )/( a.getX()-b.getX() );
    double det=1/( cos( atan(slope) ) );
    double result=0;
    for(int i=0;i<5;i++)
    {
        for(int j=0;j<area[i].opp.size();j++)
        {
            if(area[i].opp[j].Vec.getX()>a.getX()&&area[i].opp[j].Vec.getX()<b.getX())
            {
                if( (area[i].opp[j].Vec.getY()>( slope*area[i].opp[j].Vec.getX()-det ) )&&( area[i].opp[j].Vec.getY()<( slope*area[i].opp[j].Vec.getX()+det )) )
                {
                    result+=10;
                }  
            }
        }
    }
    return true;

}

// void Equili_model()
// {
 

// }


// void Defense_model()
// {

// }


void Distri_one_area(NumVec& NumVecTemp,int t)
{
    switch(t)
    {
        case 1:
            if(NumVecTemp.Vec.getX() < -9 )
            {
                NumVecTemp.serial=0;
                area[0].us.push_back(NumVecTemp);  
            }
            
            else if(NumVecTemp.Vec.getX() < -3)
            {
                NumVecTemp.serial=1;
                area[1].us.push_back(NumVecTemp);
            }
           
            else if(NumVecTemp.Vec.getX() < 3)
            {
                NumVecTemp.serial=2;
                area[2].us.push_back(NumVecTemp);
            }
            
            else if(NumVecTemp.Vec.getX() < 9)
            {
                NumVecTemp.serial=3;
                area[3].us.push_back(NumVecTemp);
            }
            
            else 
            {
                NumVecTemp.serial=4;
                area[4].us.push_back(NumVecTemp);
            }
            
        break;
        case 2:
            if(NumVecTemp.Vec.getX() < -9 )
            {
                NumVecTemp.serial=0;
                area[0].opp.push_back(NumVecTemp);
            }
            
            else if(NumVecTemp.Vec.getX() < -3)
            {
                NumVecTemp.serial=1;
                area[1].opp.push_back(NumVecTemp);
            }
            
            else if(NumVecTemp.Vec.getX() < 3)
            {
                NumVecTemp.serial=2;
                area[2].us.push_back(NumVecTemp);
            }
            
            else if(NumVecTemp.Vec.getX() < 9)
            {
                NumVecTemp.serial=3;
                area[3].opp.push_back(NumVecTemp);
            }
            
            else 
            {
                NumVecTemp.serial=4;
                area[4].opp.push_back(NumVecTemp);
            }
            
        break;
    }

}




// void evaluate_defend()
// {

// }


SkillType NaoBehavior::selectSkill() 
{

    /////////////////   distribute everyone to area and find who is the cloest for easily manage
    us_closest_distance_toball=1000.00;//ini data
    opp_closest_distance_toball=1000.00;
    ORDER=0;
    for(int i=WO_TEAMMATE1;i<WO_TEAMMATE1+11;++i)
    {
       
        if (worldModel->getUNum() == i) 
        {
            NumVecTemp.Num=i;
            NumVecTemp.Vec=worldModel->getMyPosition();
            Distri_one_area(NumVecTemp,1);
            Ball_closest_member(NumVecTemp,NumVecTemp.Vec.getDistanceTo(ball),1);

        } 
        else 
        {
            WorldObject* teammate = worldModel->getWorldObject(i);
            if (teammate->validPosition) 
            {
                NumVecTemp.Num=i;
                NumVecTemp.Vec=worldModel->getMyPosition();
                Distri_one_area(NumVecTemp,1);
                Ball_closest_member(NumVecTemp,NumVecTemp.Vec.getDistanceTo(ball),1);
            }
        else 
            {
            continue;      
            }
        }
    }//distribute our team
    
    for(int i=WO_OPPONENT1;i<WO_OPPONENT1+11;++i)
    {
        NumVecTemp.Num=i;
        NumVecTemp.Vec = worldModel->getOpponent(i);
        Distri_one_area(NumVecTemp,2);
        Ball_closest_member(NumVecTemp,NumVecTemp.Vec.getDistanceTo(ball),2);
    }//distribute oppoent

///////////////////////    select model, distribe task
    
    model_select(ball);
    ///////
    for(int i=0;i<11;++i)
    {
        if(task[i].Num==worldModel->getUNum())
        {
            if(task[i].kind==1)
            {
                VecPosition target = collisionAvoidance(true /*teammate*/, true/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, task[i].tar, true/*keepDistance*/);
                return goToTarget(target);
            }
            else if(task[i].kind==2)
            {
                return kickBall(KICK_IK,task[i].tar);
            }
            else if(task[i].kind==3)
            {
                return kickBall(KICK_FORWARD,task[i].tar);

            }
                
        }
    }
}

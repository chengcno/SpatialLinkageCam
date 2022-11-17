//
// Created by cheng on 5/8/21.
// Modified by CHENG 2022/Aug/08
//

#include "Joint.h"
#include "Mesh/MeshCreator.h"
#include "Mesh/MeshBoolean.h"
#include "Utility/HelpFunc.h"
#include "Utility/HelpDefine.h"

////////////////////////////////////////// Init //////////////////////////////////////////

Joint::Joint(Joint::JointType _type)
{
    jointType = _type;
    initPosition = Vector3d (0,0,0);
    initRotationM.setIdentity();

    N_size = CSG_NUM_N;
    M_size = CSG_NUM_N;
}

Joint::Joint(char jType)
{
    switch (jType)
    {
        case 'p':
            jointType = P_joint;
            break;
        case 'r':
            jointType = R_joint;
            break;
        case 'u':
            jointType = U_joint;
            break;
        case 'c':
            jointType = C_joint;
            break;
        case 's':
            jointType = S_joint;
            break;

        default:
            break;
    }
    initPosition = Vector3d (0,0,0);
    initRotationM.setIdentity();

    N_size = CSG_NUM_N;
    M_size = CSG_NUM_N;
}

Joint::~Joint() {

}

////////////////////////////////////////// Create //////////////////////////////////////////

void Joint::InitPose(int _linkA_id, int _linkB_id, Matrix4d initMatrix)
{
    linkA_id = _linkA_id;
    linkB_id = _linkB_id;

    initPosition = initMatrix.block(0,3,3,1);
    initRotationM = initMatrix.block(0,0,3,3);

    if(partMid != nullptr)
    {
        partMid->TransformMesh(initMatrix);
    }
    partA->TransformMesh(initMatrix);
    partB->TransformMesh(initMatrix);

    MultiplyPoint(partA_touch, initMatrix, partA_touch);
    MultiplyPoint(partB_touch, initMatrix, partB_touch);
}

void Joint::ResetPose()
{
    linkA_id = -1;
    linkB_id = -1;
    Matrix4d rM = GetTranslateMatrix(initPosition)*GetAffine(initRotationM);
    Matrix4d inM = rM.inverse();
    if(partMid != nullptr)
    {
        partMid->TransformMesh(inM);
    }
    partA->TransformMesh(inM);
    partB->TransformMesh(inM);

    MultiplyPoint(partA_touch, inM, partA_touch);
    MultiplyPoint(partB_touch, inM, partB_touch);
}

void Joint::CreateModel()
{
    switch (jointType)
    {
        case S_joint:
            CreateSjoint();
            break;
        case R_joint:
            CreateRjoint();
            break;
        case P_joint:
            CreatePjoint();
            break;
        case RR_joint:
            CreateRRjoint();
            break;
        case PR_joint:
            CreatePRjoint();
            break;
        case RRP_joint:
            CreateRRPjoint();
            break;
        case U_joint:
            CreateUjoint();
            break;
        case C_joint:
            CreateCjoint();
            break;

        default:
            break;
    }
    MeshBoolean meshBoolean;
    partAll = meshBoolean.MeshConnect(partA,  partB);
}

void Joint::CreateModel(const Vector3d &touchPA, const Vector3d &touchPB)
{
    switch (jointType)
    {
        case S_joint:
            CreateSjoint(touchPA, touchPB);
            break;
        case R_joint:
            CreateRjoint(touchPA, touchPB);
            break;
        case P_joint:
            CreatePjoint();
            break;
        case RR_joint:
            CreateRRjoint();
            break;
        case PR_joint:
            CreatePRjoint();
            break;
        case RRP_joint:
            CreateRRPjoint();
            break;
        case U_joint:
            CreateUjoint(touchPA, touchPB);
            break;
        case C_joint:
            CreateCjoint();
            break;

        default:
            break;
    }
    MeshBoolean meshBoolean;
    partAll = meshBoolean.MeshConnect(partA,  partB);
}

void Joint::CreateSimpleModel()
{
    switch (jointType)
    {
        case S_joint:
            CreateSimpleS();
            break;
        case R_joint:
            CreateSimpleR();
            break;
        case P_joint:
            CreateSimpleP();
            break;
        case U_joint:
            CreateSimpleU();
            break;
        case C_joint:
            CreateSimpleC();
            break;
        default:
            break;
    }
    MeshBoolean meshBoolean;
    partAll = meshBoolean.MeshConnect(partA,  partB);
}

// A->B: N = (0,1,0)
void Joint::CreateSjoint()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;
    double ball_out_radius = 10.0*scale;
    double ball_in_radius = 7.25*scale;
    double tol = 0.5*FAB_SCALE;
    //leftCut = -7;
    //rightCut = 5;
    double radius_cyl = 3.5*(0.4+0.6*scale);

    /// inside ball (1,0,0)
    double r_small = ball_in_radius;
    Mesh* s_in = meshCreator.CreateSphere(r_small, N_size, N_size);
    partA_touch = Vector3d( 1.6*ball_in_radius, 0 ,0);
    Mesh* s_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_cyl, N_size);
    Mesh* s_in_touch_sph = meshCreator.CreateSphere(radius_cyl, partA_touch, M_size, M_size);
    s_in_touch = meshBoolean.MeshConnect(s_in_touch, s_in_touch_sph);
    partA = meshBoolean.MeshUnion(s_in, s_in_touch);
    cout<<"finish in ball"<<endl;

    /// outside ball (1, 0, 0)
    double r_big = ball_out_radius;
    Mesh* s_out = meshCreator.CreateSphere(r_big, N_size, N_size);
    partB_touch = Vector3d(-1,0,0) + Vector3d (-0.1, sqrt(0.99), 0) + Vector3d(-0.1, 0, sqrt(0.99));
    partB_touch = partB_touch.normalized()* 2*r_small*1.25;
    Mesh* touchM_b = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, radius_cyl, N_size);
    Mesh* touchM_bSph = meshCreator.CreateSphere(radius_cyl, partB_touch, M_size,M_size);
    touchM_b = meshBoolean.MeshConnect(touchM_b, touchM_bSph);
    s_out = meshBoolean.MeshConnect(s_out, touchM_b);
    Mesh* s_out_in = meshCreator.CreateSphere(r_small + tol, N_size , N_size);
    s_out = meshBoolean.MeshMinus(s_out, s_out_in);
    cout<<"finish out ball"<<endl;

    //hole (axis, range) ~ 6
    vector<Vector3d> axis_ns;
    vector<double> ranges;
    axis_ns.emplace_back(1,0,0);
    ranges.emplace_back(M_PI*0.75);
    axis_ns.emplace_back(-1,0,0);
    ranges.emplace_back(M_PI*0.2);
    Vector3d Ni = Vector3d (0,1,0);
    MultiplyVector(Ni, GetRotationMatrix(M_PI*0.1,Vector3d(0,0,1)), Ni);
    int n_s = 4;
    for (int i=0;i<n_s;i++)
    {
        axis_ns.push_back(Ni);
        ranges.push_back(M_PI*0.2);
        MultiplyVector(Ni, GetRotationMatrix(2*M_PI/n_s,Vector3d(1,0,0)), Ni);
    }
    for(int i=0;i<6;i++)
    {
        Vector3d axis_n = axis_ns[i];
        double rangeAngle = ranges[i];
        Mesh *cone = meshCreator.CreateCone(Vector3d(0, 0, 0), axis_n, ball_out_radius * 1.1, rangeAngle, N_size);
        double r_open, dis_open;
        r_open = (tol + ball_in_radius) * sin(rangeAngle / 2);
        dis_open = sqrt(pow(tol + ball_in_radius, 2) - pow(r_open, 2)) - sqrt(pow(ball_in_radius, 2) - pow(r_open, 2));
        //cout << "r_open = " << r_open << " dis_open = " << dis_open << endl;
        s_out = meshBoolean.MeshMinus(s_out, cone);
    }
    cout<<"finish out ball holes"<<endl;

    partB = s_out;
}

void Joint::CreateSjoint(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;
    double ball_out_radius = 10.0*scale;
    double ball_in_radius = 7.25*scale;
    double tol = 0.5*FAB_SCALE;
    //leftCut = -7;
    //rightCut = 5;
    double radius_cyl = 3.5*(0.4+0.6*scale);

    /// inside ball (1,0,0)
    double r_small = ball_in_radius;
    Mesh* s_in = meshCreator.CreateSphere(r_small, N_size, N_size);
    partA_touch = touchPA;//Vector3d( 2*ball_in_radius, 0 ,0);
    Mesh* s_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_cyl, N_size);
    Mesh* s_in_touch_sph = meshCreator.CreateSphere(radius_cyl, partA_touch, M_size, M_size);
    s_in_touch = meshBoolean.MeshConnect(s_in_touch, s_in_touch_sph);
    partA = meshBoolean.MeshUnion(s_in, s_in_touch);
    cout<<"finish in ball"<<endl;

    /// outside ball (1, 0, 0)
    double r_big = ball_out_radius;
    Mesh* s_out = meshCreator.CreateSphere(r_big, N_size, N_size);
    //partB_touch = Vector3d(-1,0,0) + Vector3d (-0.1, sqrt(0.99), 0) + Vector3d(-0.1, 0, sqrt(0.99));
    //partB_touch = partB_touch.normalized()* 2*r_small;
    partB_touch = touchPB.normalized()* 2*r_small;
    Mesh* touchM_b = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, radius_cyl, N_size);
    Mesh* touchM_bSph = meshCreator.CreateSphere(radius_cyl, partB_touch, M_size,M_size);
    touchM_b = meshBoolean.MeshConnect(touchM_b, touchM_bSph);
    s_out = meshBoolean.MeshConnect(s_out, touchM_b);
    Mesh* s_out_in = meshCreator.CreateSphere(r_small + tol, N_size , N_size);
    s_out = meshBoolean.MeshMinus(s_out, s_out_in);
    cout<<"finish out ball"<<endl;

    //hole (axis, range) ~ 6
    vector<Vector3d> axis_ns;
    vector<double> ranges;
    axis_ns.emplace_back(1,0,0);
    ranges.emplace_back(M_PI*0.75);
    axis_ns.emplace_back(-1,0,0);
    ranges.emplace_back(M_PI*0.2);
    Vector3d Ni = Vector3d (0,1,0);
    MultiplyVector(Ni, GetRotationMatrix(M_PI*0.1,Vector3d(0,0,1)), Ni);
    int n_s = 4;
    for (int i=0;i<n_s;i++)
    {
        axis_ns.push_back(Ni);
        ranges.push_back(M_PI*0.2);
        MultiplyVector(Ni, GetRotationMatrix(2*M_PI/n_s,Vector3d(1,0,0)), Ni);
    }
    for(int i=0;i<6;i++)
    {
        Vector3d axis_n = axis_ns[i];
        double rangeAngle = ranges[i];
        Mesh *cone = meshCreator.CreateCone(Vector3d(0, 0, 0), axis_n, ball_out_radius * 1.1, rangeAngle, N_size);
        double r_open, dis_open;
        r_open = (tol + ball_in_radius) * sin(rangeAngle / 2);
        dis_open = sqrt(pow(tol + ball_in_radius, 2) - pow(r_open, 2)) - sqrt(pow(ball_in_radius, 2) - pow(r_open, 2));
        //cout << "r_open = " << r_open << " dis_open = " << dis_open << endl;
        s_out = meshBoolean.MeshMinus(s_out, cone);
    }
    cout<<"finish out ball holes"<<endl;

    partB = s_out;
}

void Joint::CreateSimpleS()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;
    double ball_out_radius = 10.0*scale;
    double ball_in_radius = 7.25*scale;
    double tol = 0.25*scale;
    //leftCut = -7;
    //rightCut = 5;
    double radius_cyl = 3.5*scale;

    /// inside ball (1,0,0)
    double r_small = ball_in_radius;
    Mesh* s_in = meshCreator.CreateSphere(r_small - tol, N_size, N_size);
    partA_touch = Vector3d( 2*ball_in_radius, 0 ,0);
    Mesh* s_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_cyl, N_size);
    Mesh* s_in_touch_sph = meshCreator.CreateSphere(radius_cyl, partA_touch, M_size, M_size);
    s_in_touch = meshBoolean.MeshConnect(s_in_touch, s_in_touch_sph);
    partA = meshBoolean.MeshUnion(s_in, s_in_touch);
    cout<<"finish in ball"<<endl;

    /// outside ball (1, 0, 0)
    double r_big = ball_out_radius;
    Mesh* s_out = meshCreator.CreateSphere(r_big, N_size, N_size);
    partB_touch = Vector3d(-1,0,0) + Vector3d (-0.1, sqrt(0.99), 0) + Vector3d(-0.1, 0, sqrt(0.99));
    partB_touch = partB_touch.normalized()* 2*r_small;
    Mesh* touchM_b = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, radius_cyl, N_size);
    Mesh* touchM_bSph = meshCreator.CreateSphere(radius_cyl, partB_touch, M_size,M_size);
    touchM_b = meshBoolean.MeshConnect(touchM_b, touchM_bSph);
    s_out = meshBoolean.MeshConnect(s_out, touchM_b);
    Mesh* s_out_in = meshCreator.CreateSphere(r_small + tol, N_size , N_size);
    s_out = meshBoolean.MeshMinus(s_out, s_out_in);
    cout<<"finish out ball"<<endl;

    //hole (axis, range) ~ 1
    vector<Vector3d> axis_ns;
    vector<double> ranges;
    axis_ns.emplace_back(1,0,0);
    ranges.emplace_back(M_PI*0.85);

    for(int i=0;i<1;i++)
    {
        Vector3d axis_n = axis_ns[i];
        double rangeAngle = ranges[i];
        Mesh *cone = meshCreator.CreateCone(Vector3d(0, 0, 0), axis_n, ball_out_radius * 1.1, rangeAngle, N_size);
        double r_open, dis_open;
        r_open = (tol + ball_in_radius) * sin(rangeAngle / 2);
        dis_open = sqrt(pow(tol + ball_in_radius, 2) - pow(r_open, 2)) - sqrt(pow(ball_in_radius, 2) - pow(r_open, 2));
        //cout << "r_open = " << r_open << " dis_open = " << dis_open << endl;
        s_out = meshBoolean.MeshMinus(s_out, cone);
    }
    cout<<"finish out ball holes"<<endl;

    partB = s_out;
}

// N = (0,1,0);
void Joint::CreateRjoint()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;

    double Radius_large_cyl = 10* scale;
    double tol1 = 0.45; /// 0.3
    double tol2 = 0.5;
    double thick  = 3.0 *scale; ///2.5
    double Radius_small_cyl = Radius_large_cyl - thick - tol2;
    double height_small = 7.5*scale; ///6
    double height_hat = 2.5*scale;
    double height_large = height_small + 2*tol1 + 2*height_hat; //13.3
    double radius_fol = 3.5*scale; ///2.5

    double wh_out = 1.5*scale;
    //int Num_out = 4;

    int N = N_size;
    double hc = height_small;
    double tc = height_hat;

    /// cyl in & touch
    partA_touch = Vector3d (0, height_large,0);
    Mesh* cyl_in = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tc ,0), Vector3d (0,hc/2+tc , 0), Radius_small_cyl, N);
    Mesh* cyl_in_h1 = meshCreator.CreateCylinder(Vector3d(0,hc/2+tol1,0), Vector3d (0,hc/2+tol1+tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_h2 = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tol1,0), Vector3d (0,-hc/2-tol1-tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_fol, N);
    Mesh* cyl_in_touch_sph = meshCreator.CreateSphere(radius_fol, partA_touch, M_size, M_size);
    cyl_in_touch = meshBoolean.MeshConnect(cyl_in_touch, cyl_in_touch_sph);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_touch);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_h1);
    cyl_in = meshBoolean.MeshUnion(cyl_in, cyl_in_h2);
    cout<<"finish create R joint inside"<<endl;

    /// cyl out & touch
    partB_touch = Vector3d (0,0,-Radius_large_cyl*1.5);
    Mesh* cyl_out = meshCreator.CreateCylinder(Vector3d(0,-hc/2,0), Vector3d (0,hc/2, 0),Radius_large_cyl, N);
    Mesh* cyl_out_touch = meshCreator.CreateCylinder(Vector3d (0, 0,0),partB_touch, radius_fol,N);
    Mesh* cyl_out_touch_sph = meshCreator.CreateSphere(radius_fol, partB_touch, M_size, M_size);
    cyl_out_touch = meshBoolean.MeshConnect(cyl_out_touch, cyl_out_touch_sph);
    cyl_out = meshBoolean.MeshConnect(cyl_out,cyl_out_touch);
    Mesh* cyl_out_in = meshCreator.CreateCylinder(Vector3d(0,-hc,0), Vector3d (0,hc, 0),Radius_large_cyl-thick, N);
    cyl_out = meshBoolean.MeshMinus(cyl_out, cyl_out_in);
    cout<<"finish create R joint outside"<<endl;

    /// create hole in
    double wh = 1.5*scale;
    Mesh* FanO = meshCreator.CreateFan(Radius_small_cyl + wh, tol1 + height_hat, -0.1-M_PI_4/2, M_PI_4/2+0.1, N_size);
    Mesh* FanI = meshCreator.CreateFan(Radius_small_cyl, tol1*2+height_hat, -0.2-M_PI_4/2, M_PI_4/2+0.2, N_size);
    Mesh* holeM = meshBoolean.MeshMinus(FanO, FanI);
    holeM ->TransformMesh(GetTranslateMatrix(Vector3d(0, height_small/2 + tol1 + height_hat/2, 0)));
    for(int i=0;i<4;i++)
    {
        Matrix4d rM = GetRotationMatrix(M_PI_2, Vector3d(0, 1, 0));
        holeM->TransformMesh(rM);
        cyl_in = meshBoolean.MeshMinus(cyl_in, holeM);
    }
    holeM ->TransformMesh(GetTranslateMatrix(Vector3d(0, -height_small - 2*tol1 - height_hat, 0)));
    for(int i=0;i<4;i++)
    {
        Matrix4d rM = GetRotationMatrix(M_PI_2, Vector3d(0, 1, 0));
        holeM->TransformMesh(rM);
        cyl_in = meshBoolean.MeshMinus(cyl_in, holeM);
    }
    cout<<"finish create R joint inside holes"<<endl;


    /// create hole out

    double theta0 = 1.5* asin(radius_fol/Radius_large_cyl);
    double theta1 = M_PI_2-theta0;
    Mesh* holeM2 = meshCreator.CreateFan(Radius_large_cyl*1.1, wh_out*2,theta0, theta1, N_size);
    for(int i=0;i<4;i++)
    {
        cyl_out = meshBoolean.MeshMinus(cyl_out, holeM2);
        holeM2->TransformMesh(GetRotationMatrix(M_PI_2, Vector3d(0,1,0)));
    }
    cout<<"finish create R joint outside holes"<<endl;

    partA = cyl_in;
    partB = cyl_out;
}

void Joint::CreateRjoint(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;

    double Radius_large_cyl = 10* scale;
    double tol1 = 0.45 * FAB_SCALE; /// 0.3
    double tol2 = 0.5 * FAB_SCALE;
    double thick  = 3.0 *scale; ///2.5
    double Radius_small_cyl = Radius_large_cyl - thick - tol2;
    double height_small = 7.5*scale; ///6
    double height_hat = 2.5*scale;
    double height_large = height_small + 2*tol1 + 2*height_hat; //13.3
    double radius_fol = 3.5*scale; ///2.5

    double wh_out = 1.5*scale;
    //int Num_out = 4;

    int N = N_size;
    double hc = height_small;
    double tc = height_hat;

    /// cyl in & touch
    partA_touch = touchPA;//Vector3d (0, height_large,0);
    Mesh* cyl_in = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tc ,0), Vector3d (0,hc/2+tc , 0), Radius_small_cyl, N);
    Mesh* cyl_in_h1 = meshCreator.CreateCylinder(Vector3d(0,hc/2+tol1,0), Vector3d (0,hc/2+tol1+tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_h2 = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tol1,0), Vector3d (0,-hc/2-tol1-tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_fol, N);
    Mesh* cyl_in_touch_sph = meshCreator.CreateSphere(radius_fol, partA_touch, M_size, M_size);
    cyl_in_touch = meshBoolean.MeshConnect(cyl_in_touch, cyl_in_touch_sph);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_touch);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_h1);
    cyl_in = meshBoolean.MeshUnion(cyl_in, cyl_in_h2);
    cout<<"finish create R joint inside"<<endl;

    /// cyl out & touch
    partB_touch = touchPB;//Vector3d (0,0,-Radius_large_cyl*1.5);
    Mesh* cyl_out = meshCreator.CreateCylinder(Vector3d(0,-hc/2,0), Vector3d (0,hc/2, 0),Radius_large_cyl, N);
    Mesh* cyl_out_touch = meshCreator.CreateCylinder(Vector3d (0, 0,0),partB_touch, radius_fol,N);
    Mesh* cyl_out_touch_sph = meshCreator.CreateSphere(radius_fol, partB_touch, M_size, M_size);
    cyl_out_touch = meshBoolean.MeshConnect(cyl_out_touch, cyl_out_touch_sph);
    cyl_out = meshBoolean.MeshConnect(cyl_out,cyl_out_touch);
    Mesh* cyl_out_in = meshCreator.CreateCylinder(Vector3d(0,-hc,0), Vector3d (0,hc, 0),Radius_large_cyl-thick, N);
    cyl_out = meshBoolean.MeshMinus(cyl_out, cyl_out_in);
    cout<<"finish create R joint outside"<<endl;

    /// create hole in
    double wh = 1.5*scale;
    Mesh* FanO = meshCreator.CreateFan(Radius_small_cyl + wh, tol1 + height_hat, -0.1-M_PI_4/2, M_PI_4/2+0.1, N_size);
    Mesh* FanI = meshCreator.CreateFan(Radius_small_cyl, tol1*2+height_hat, -0.2-M_PI_4/2, M_PI_4/2+0.2, N_size);
    Mesh* holeM = meshBoolean.MeshMinus(FanO, FanI);
    holeM ->TransformMesh(GetTranslateMatrix(Vector3d(0, height_small/2 + tol1 + height_hat/2, 0)));
    for(int i=0;i<4;i++)
    {
        Matrix4d rM = GetRotationMatrix(M_PI_2, Vector3d(0, 1, 0));
        holeM->TransformMesh(rM);
        cyl_in = meshBoolean.MeshMinus(cyl_in, holeM);
    }
    holeM ->TransformMesh(GetTranslateMatrix(Vector3d(0, -height_small - 2*tol1 - height_hat, 0)));
    for(int i=0;i<4;i++)
    {
        Matrix4d rM = GetRotationMatrix(M_PI_2, Vector3d(0, 1, 0));
        holeM->TransformMesh(rM);
        cyl_in = meshBoolean.MeshMinus(cyl_in, holeM);
    }
    cout<<"finish create R joint inside holes"<<endl;


    /// create hole out

    double theta0 = 1.5* asin(radius_fol/Radius_large_cyl);
    double theta1 = M_PI_2-theta0;
    Mesh* holeM2 = meshCreator.CreateFan(Radius_large_cyl*1.1, wh_out*2,theta0, theta1, N_size);
    for(int i=0;i<4;i++)
    {
        cyl_out = meshBoolean.MeshMinus(cyl_out, holeM2);
        holeM2->TransformMesh(GetRotationMatrix(M_PI_2, Vector3d(0,1,0)));
    }
    cout<<"finish create R joint outside holes"<<endl;

    partA = cyl_in;
    partB = cyl_out;
}
// N = (0,1,0);
void Joint::CreateSimpleR()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;

    double Radius_large_cyl = 10* scale;
    double tol1 = 0.3;
    double tol2 = 0.5;
    double thick  = 2.5 *scale;
    double Radius_small_cyl = Radius_large_cyl - thick - tol2;
    double height_small = 6*scale;
    double height_hat = 2.5*scale;
    double height_large = height_small + 2*tol1 + 2*height_hat;
    double radius_fol = 3.5*scale;

    double wh_out = 1.5*scale;

    int N = N_size;
    double hc = height_small;
    double tc = height_hat;

    /// cyl in & touch
    partA_touch = Vector3d (0, height_large,0);
    Mesh* cyl_in = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tc ,0), Vector3d (0,hc/2+tc , 0), Radius_small_cyl, N);
    Mesh* cyl_in_h1 = meshCreator.CreateCylinder(Vector3d(0,hc/2+tol1,0), Vector3d (0,hc/2+tol1+tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_h2 = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tol1,0), Vector3d (0,-hc/2-tol1-tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_fol, N);
    Mesh* cyl_in_touch_sph = meshCreator.CreateSphere(radius_fol, partA_touch, M_size, M_size);
    cyl_in_touch = meshBoolean.MeshConnect(cyl_in_touch, cyl_in_touch_sph);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_touch);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_h1);
    cyl_in = meshBoolean.MeshUnion(cyl_in, cyl_in_h2);
    cout<<"finish create R joint inside"<<endl;

    /// cyl out & touch
    partB_touch = Vector3d (0,0,-Radius_large_cyl*1.5);
    Mesh* cyl_out = meshCreator.CreateCylinder(Vector3d(0,-hc/2,0), Vector3d (0,hc/2, 0),Radius_large_cyl, N);
    Mesh* cyl_out_touch = meshCreator.CreateCylinder(Vector3d (0, 0,0),partB_touch, radius_fol,N);
    Mesh* cyl_out_touch_sph = meshCreator.CreateSphere(radius_fol, partB_touch, M_size, M_size);
    cyl_out_touch = meshBoolean.MeshConnect(cyl_out_touch, cyl_out_touch_sph);
    cyl_out = meshBoolean.MeshConnect(cyl_out,cyl_out_touch);
    Mesh* cyl_out_in = meshCreator.CreateCylinder(Vector3d(0,-hc,0), Vector3d (0,hc, 0),Radius_large_cyl-thick, N);
    cyl_out = meshBoolean.MeshMinus(cyl_out, cyl_out_in);
    cout<<"finish create R joint outside"<<endl;

    partA = cyl_in;
    partB = cyl_out;
}

void Joint::CreateCjoint()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;
    ///
    double rl = 3.5*scale;   //6
    double Rl = 7.5*scale; //10
    double lengT = 30*scale;
    double thick = 6*scale;
    double fr = 3.5*scale;
    double tol = 0.3;

    partA = meshCreator.CreateCylinder(Vector3d(0, -lengT*0.6 ,0), Vector3d(0,lengT ,0), rl, N_size);
    partA_touch = Vector3d (0, lengT-fr ,0);
    Mesh* touchASph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    partA = meshBoolean.MeshUnion(partA, touchASph);

    partB = meshCreator.CreateCylinder(Vector3d(0,-thick, 0), Vector3d(0,thick,0), Rl, N_size);
    partB_touch = Vector3d ( 2*Rl,0,0);
    Mesh* touchB = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0,0), fr,N_size);
    Mesh* touchBsph = meshCreator.CreateSphere(fr,partB_touch, M_size, M_size);
    touchB = meshBoolean.MeshConnect(touchB, touchBsph);
    partB = meshBoolean.MeshConnect(partB, touchB);

    Mesh* fanM = meshCreator.CreateFan(Rl+0.1, thick/1.6, -M_PI_4, M_PI_4, N_size);
    partB = meshBoolean.MeshMinus(partB, fanM);
    fanM->TransformMesh(GetRotationMatrix(Vector3d(M_PI, 0, 0)));
    partB = meshBoolean.MeshMinus(partB, fanM);

    Mesh* cutM = meshCreator.CreateCylinder(Vector3d(0,-lengT, 0), Vector3d(0,lengT, 0 ), rl+tol, N_size);
    partB = meshBoolean.MeshMinus(partB, cutM);

}

void Joint::CreateSimpleC()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;
    ///
    double rl = 3.5*scale;
    double Rl = 7.5*scale;
    double lengT = 40*scale;
    double thick = 5*scale;
    double fr = 2.5*scale;
    double tol = 0.5;

    //partA = meshCreator.CreateCylinder(Vector3d(0, -lengT ,0), Vector3d(0,lengT ,0), rl, N_size);
    partA = meshCreator.CreateCylinder(Vector3d(0, -lengT*0.6 ,0), Vector3d(0,lengT ,0), rl, N_size);
    partA_touch = Vector3d (0, lengT-fr ,0);


    partB = meshCreator.CreateCylinder(Vector3d(0,-thick, 0), Vector3d(0,thick,0), Rl, N_size);
    partB_touch = Vector3d ( 2*Rl,0,0);
    Mesh* touchB = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0,0), fr,N_size);
    Mesh* touchBsph = meshCreator.CreateSphere(fr,partB_touch, M_size, M_size);
    touchB = meshBoolean.MeshConnect(touchB, touchBsph);
    partB = meshBoolean.MeshConnect(partB, touchB);

    Mesh* cutM = meshCreator.CreateCylinder(Vector3d(0,-lengT, 0), Vector3d(0,lengT, 0 ), rl+tol, N_size);
    partB = meshBoolean.MeshMinus(partB, cutM);
}

// T = (1,0,0); N = (0,1,0);
void Joint::CreatePjoint()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 1.0;
    Vector3d SliderSize = Vector3d (8, 8, 8) * scale; ///5,8,8
    double lengT = 44*scale;
    double lengY, lengZ;
    double tol = 0.3;
    double rl = 3.5*scale;
    lengY = lengZ = 3.5*scale;

    partA = meshCreator.CreateCuboid(Vector3d (-lengT*0.5,-lengY,-lengZ), Vector3d (lengT,lengY,lengZ));
    partA_touch = Vector3d(lengT - lengY,0,0);
    Mesh* touchASph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    partA = meshBoolean.MeshUnion(partA, touchASph);

    partB = meshCreator.CreateCuboid(-SliderSize, SliderSize);
    partB_touch = Vector3d (0,0,16)*scale;
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, rl ,N_size);
    Mesh* touchMsph = meshCreator.CreateSphere(rl ,partB_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchMsph);
    partB = meshBoolean.MeshConnect(partB, touchM);
    Mesh* cutM = meshCreator.CreateCuboid(Vector3d (-lengT,-lengY-tol,-lengZ-tol), Vector3d (lengT,lengY+tol,lengZ+tol));
    partB = meshBoolean.MeshMinus(partB, cutM);

    Mesh* holeM = meshCreator.CreateCuboid(Vector3d(-SliderSize.x()/2, -10*scale, -SliderSize.z()/2), Vector3d(SliderSize.x()/2, 10*scale, SliderSize.z()/2));
    partB = meshBoolean.MeshMinus(partB, holeM);

}

void Joint::CreateSimpleP()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;
    Vector3d SliderSize = Vector3d (5, 8, 8) * scale;
    double lengT = 40*scale;
    double lengY, lengZ;
    double tol = 0.4;
    double rl = 3.5*scale;
    lengY = lengZ = 3*scale;

    //partA = meshCreator.CreateCuboid(Vector3d (-lengT,-lengY,-lengZ), Vector3d (lengT,lengY,lengZ));
    partA = meshCreator.CreateCuboid(Vector3d (-lengT*0.5,-lengY,-lengZ), Vector3d (lengT,lengY,lengZ));
    partA_touch = Vector3d(lengT - lengY,0,0);

    partB = meshCreator.CreateCuboid(-SliderSize, SliderSize);
    partB_touch = Vector3d (0,0,16)*scale;
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, rl ,N_size);
    Mesh* touchMsph = meshCreator.CreateSphere(rl ,partB_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchMsph);
    partB = meshBoolean.MeshConnect(partB, touchM);
    Mesh* cutM = meshCreator.CreateCuboid(Vector3d (-lengT,-lengY-tol,-lengZ-tol), Vector3d (lengT,lengY+tol,lengZ+tol));
    partB = meshBoolean.MeshMinus(partB, cutM);

}

void Joint::CreateRRjoint()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double thick = 0.1;
    Mesh* CM = meshCreator.CreateCylinder(Vector3d(0,0, -thick), Vector3d(0,0,thick), 1, 40);
    Mesh* cM = meshCreator.CreateCylinder(Vector3d(0,0,-2*thick), Vector3d(0,0,2*thick), 1-thick, 30);
    Mesh* cutM = meshCreator.CreateCuboid(Vector3d(0, -2, -1), Vector3d(2, 2, 1));
    partA = meshBoolean.MeshMinus(CM, cM);
    partA = meshBoolean.MeshMinus(partA, cutM);
    double sq = 0.2;
    partA->TransformMesh(GetTranslateMatrix(Vector3d(-sq*0.99,0,0)));
    Mesh* usqM = meshCreator.CreateCuboid(Vector3d(-sq, 1-thick, -sq), Vector3d (sq, 1, sq));
    Mesh* dsqM = meshCreator.CreateCuboid(Vector3d(-sq, -1, -sq), Vector3d(sq, -1+thick, sq));
    usqM = meshBoolean.MeshUnion(usqM, dsqM);
    partA = meshBoolean.MeshUnion(partA, usqM);

    double cr = 0;
    partMid = meshCreator.CreateCylinder(Vector3d(0, -1-thick, 0), Vector3d(0, 1+thick, 0), sq/2, 20);
    Mesh* crM = meshCreator.CreateCylinder(Vector3d(0,0,-1-thick), Vector3d(0,0,1+thick), sq/2, 20);
    crM->TransformMesh(GetRotationMatrix(Vector3d(cr,0,0)));
    partMid = meshBoolean.MeshUnion(partMid, crM);

    partA = meshBoolean.MeshMinus(partA, partMid);
    partB = partA->DeepCopy();
    partB->TransformMesh(GetRotationMatrix(Vector3d(M_PI_2,M_PI,0)));
    partB->TransformMesh(GetRotationMatrix(Vector3d(cr,0,0)));
}

void Joint::CreatePRjoint()
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double thick = 0.1;
    partA = meshCreator.CreateCuboid(Vector3d (-2,-thick,-0.8), Vector3d (2,thick,0.8));
    Mesh* cutM = meshCreator.CreateCuboid(Vector3d(-1,-thick*2, -thick), Vector3d(1,thick*2,thick));
    cutM->TransformMesh(GetRotationMatrix(Vector3d(0, M_PI/8.0, 0)));
    partA = meshBoolean.MeshMinus(partA, cutM);

    partB = meshCreator.CreateCylinder(Vector3d(0,-thick*1.2, 0), Vector3d(0,thick*1.2,0), thick, 20);
    Mesh* hatM = meshCreator.CreateCylinder(Vector3d(0, thick, 0), Vector3d(0,thick*1.5,0), thick*1.5,30);
    Mesh* barM = meshCreator.CreateCuboid(Vector3d(-thick*2, -thick*2, -thick*1.5), Vector3d(2*thick, -thick, 2));
    hatM = meshBoolean.MeshUnion(hatM, barM);
    partB = meshBoolean.MeshUnion(partB, hatM);
}

void Joint::CreateRRPjoint()
{
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    partA = meshCreator.CreateCuboid(Vector3d(-0.3, -0.3, -0.2), Vector3d(0.3, 0.3, 0.2));
    partB = meshCreator.CreateCuboid(Vector3d(-0.1,-0.1,-1), Vector3d(0.1,0.1,1));
    partMid = meshCreator.CreateSphere(0.2,20,20);

    //partA = meshBoolean.MeshMinus(partA, partMid);

    //Mesh* cutM = meshCreator.CreateCuboid(Vector3d(-0.1, -1, -0.05), Vector3d(0.1, 1, 0.05));
    //partMid = meshBoolean.MeshMinus(partMid, cutM);
    //partMid = meshBoolean.MeshMinus(partMid, partB);
}

void Joint::CreateUjoint()
{
    partMid = nullptr;
    double scale = 1.0;
    double inBallRadius = 7.5*scale;
    double tol1 = 0.5;
    double tol2 = 0.25;
    double lengthCube = 20.0*scale;
    double leftCube = -4.2*scale;
    double rightCube = 4.2*scale;
    double grooveDepth = 3 *scale;
    double pinRadius = 1.5 *scale;
    double rl = 3.5 *scale; ///

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    int N = N_size;

    /// inside ball
    // fan lie on x-z plane, thick deps on y-axis
    Mesh* ballMesh = meshCreator.CreateSphere(inBallRadius, Vector3d(0,0,0), N, N);
    Mesh* FanL = meshCreator.CreateFan(inBallRadius+tol1, 2*pinRadius, -M_PI/1.5, M_PI/1.5, N);
    Mesh* FanS = meshCreator.CreateFan(inBallRadius - grooveDepth - tol1, 2*pinRadius+0.1, -M_PI/1.2, M_PI/1.2, N);
    FanL = meshBoolean.MeshMinus(FanL, FanS);
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    FanL->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI, 0)));
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    // touch
    //partA_touch = Vector3d(0,2*inBallRadius,0);
    partA_touch = Vector3d(0.8*inBallRadius,2*inBallRadius,0.5*inBallRadius);
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch,rl,N);
    Mesh* touchSph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchSph);
    ballMesh = meshBoolean.MeshUnion(ballMesh, touchM);
    cout<<"finish U joint inside ball"<<endl;

    /// out supp
    Mesh* suppMesh = meshCreator.CreateCuboid(Vector3d(-lengthCube/2, leftCube, -lengthCube/2), Vector3d(lengthCube/2, rightCube, lengthCube/2));
    Mesh* cutBall = meshCreator.CreateSphere(inBallRadius+tol1, Vector3d(0,0,0), N, N);
    suppMesh = meshBoolean.MeshMinus(suppMesh, cutBall);
    // hole x-axis
    Mesh* holeM = meshCreator.CreateCuboid(Vector3d(-lengthCube, leftCube/2, -lengthCube/4), Vector3d(lengthCube, rightCube/2, lengthCube/4));
    suppMesh = meshBoolean.MeshMinus(suppMesh, holeM);
    // pin z-axis
    Mesh* pin1 = meshCreator.CreateCylinder(Vector3d(0,0,inBallRadius+2*tol1-grooveDepth), Vector3d(0,0, inBallRadius+2*tol1), pinRadius-tol2, N);
    suppMesh = meshBoolean.MeshConnect(suppMesh, pin1);
    pin1 ->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI,0)));
    suppMesh = meshBoolean.MeshUnion(suppMesh, pin1);
    // touch
    partB_touch = Vector3d(0,0, -inBallRadius-lengthCube/2);
    Mesh* touchSupp = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0, -lengthCube/2 + tol2), rl , N);
    Mesh* touchSuppSPh = meshCreator.CreateSphere(rl, partB_touch, M_size, M_size);
    touchSupp = meshBoolean.MeshConnect(touchSupp, touchSuppSPh);
    suppMesh = meshBoolean.MeshUnion(suppMesh, touchSupp);
    cout<<"finish U joint outside supp"<<endl;

    /// spin around(0, 0, 1); rotate (0,1,0),
    Matrix4d rM = GetRotationMatrix(Vector3d(M_PI_2, 0, 0));
    ballMesh->TransformMesh(rM);
    suppMesh->TransformMesh(rM);
    MultiplyPoint(partA_touch, rM, partA_touch);
    MultiplyPoint(partB_touch, rM, partB_touch);
    partA = ballMesh;
    partB = suppMesh;
}

void Joint::CreateUjoint(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    double scale = 1.0;
    double inBallRadius = 7.5*scale;
    double tol1 = 0.5 * FAB_SCALE;
    double tol2 = 0.3 * FAB_SCALE;  ///
    double lengthCube = 20.0*scale;
    double leftCube = -4.2*scale;
    double rightCube = 4.2*scale;
    double grooveDepth = 3 *scale;
    double pinRadius = 1.5 *scale;
    double rl = 3.5 *scale; ///

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    int N = N_size;

    /// inside ball
    // fan lie on x-z plane, thick deps on y-axis
    Mesh* ballMesh = meshCreator.CreateSphere(inBallRadius, Vector3d(0,0,0), N, N);
    Mesh* FanL = meshCreator.CreateFan(inBallRadius+tol1, 2*pinRadius, -M_PI/1.5, M_PI/1.5, N);
    Mesh* FanS = meshCreator.CreateFan(inBallRadius - grooveDepth - tol1, 2*pinRadius+0.1, -M_PI/1.2, M_PI/1.2, N);
    FanL = meshBoolean.MeshMinus(FanL, FanS);
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    FanL->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI, 0)));
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    // touch
    //partA_touch = Vector3d(0,2*inBallRadius,0);
    partA_touch = touchPA;
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch,rl,N);
    Mesh* touchSph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchSph);
    ballMesh = meshBoolean.MeshUnion(ballMesh, touchM);
    cout<<"finish U joint inside ball"<<endl;

    /// out supp
    Mesh* suppMesh = meshCreator.CreateCuboid(Vector3d(-lengthCube/2, leftCube, -lengthCube/2), Vector3d(lengthCube/2, rightCube, lengthCube/2));
    // touch
    partB_touch = touchPB;
    Mesh* touchSupp = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0, 0), rl , N);
    Mesh* touchSuppSPh = meshCreator.CreateSphere(rl, partB_touch, M_size, M_size);
    touchSupp = meshBoolean.MeshConnect(touchSupp, touchSuppSPh);
    suppMesh = meshBoolean.MeshUnion(suppMesh, touchSupp);

    Mesh* cutBall = meshCreator.CreateSphere(inBallRadius+tol1, Vector3d(0,0,0), N, N);
    suppMesh = meshBoolean.MeshMinus(suppMesh, cutBall);
    // hole x-axis
    Mesh* holeM = meshCreator.CreateCuboid(Vector3d(-lengthCube, leftCube/2, -lengthCube/4), Vector3d(lengthCube, rightCube/2, lengthCube/4));
    suppMesh = meshBoolean.MeshMinus(suppMesh, holeM);
    // pin z-axis
    Mesh* pin1 = meshCreator.CreateCylinder(Vector3d(0,0,inBallRadius+2*tol1-grooveDepth), Vector3d(0,0, inBallRadius+2*tol1), pinRadius-tol2, N);
    suppMesh = meshBoolean.MeshConnect(suppMesh, pin1);
    pin1 ->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI,0)));
    suppMesh = meshBoolean.MeshUnion(suppMesh, pin1);

    cout<<"finish U joint outside supp"<<endl;

    /// spin around(0, 0, 1); rotate (0,1,0),
    Matrix4d rM = GetRotationMatrix(Vector3d(M_PI_2, 0, 0));
    ballMesh->TransformMesh(rM);
    suppMesh->TransformMesh(rM);
    MultiplyPoint(partA_touch, rM, partA_touch);
    MultiplyPoint(partB_touch, rM, partB_touch);
    partA = ballMesh;
    partB = suppMesh;
}

void Joint::CreateSimpleU()
{
    partMid = nullptr;
    double scale = 0.6;
    double inBallRadius = 7.5*scale;
    double tol1 = 0.5;
    double tol2 = 0.25;
    double lengthCube = 20.0*scale;
    double leftCube = -4.2*scale;
    double rightCube = 4.2*scale;
    double grooveDepth = 3 *scale;
    double pinRadius = 1.5 *scale;
    double rl = 3.5*scale;

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    int N = N_size;

    /// inside ball
    // fan lie on x-z plane, thick deps on y-axis
    Mesh* ballMesh = meshCreator.CreateSphere(inBallRadius, Vector3d(0,0,0), N, N);
    Mesh* FanL = meshCreator.CreateFan(inBallRadius+tol1, 2*pinRadius, -M_PI/1.5, M_PI/1.5, N);
    Mesh* FanS = meshCreator.CreateFan(inBallRadius - grooveDepth - tol1, 2*pinRadius+0.1, -M_PI/1.2, M_PI/1.2, N);
    FanL = meshBoolean.MeshMinus(FanL, FanS);
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    FanL->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI, 0)));
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    // touch
    partA_touch = Vector3d(0,2*inBallRadius,0);
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch,rl,N);
    Mesh* touchSph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchSph);
    ballMesh = meshBoolean.MeshUnion(ballMesh, touchM);
    cout<<"finish U joint inside ball"<<endl;

    /// out supp
    Mesh* suppMesh = meshCreator.CreateCuboid(Vector3d(-lengthCube/2, leftCube, -lengthCube/2), Vector3d(lengthCube/2, rightCube, lengthCube/2));
    Mesh* cutBall = meshCreator.CreateSphere(inBallRadius+tol1, Vector3d(0,0,0), N, N);
    suppMesh = meshBoolean.MeshMinus(suppMesh, cutBall);
    // hole x-axis
    //Mesh* holeM = meshCreator.CreateCuboid(Vector3d(-lengthCube, leftCube/2, -lengthCube/4), Vector3d(lengthCube, rightCube/2, lengthCube/4));
    //suppMesh = meshBoolean.MeshMinus(suppMesh, holeM);
    // pin z-axis
    Mesh* pin1 = meshCreator.CreateCylinder(Vector3d(0,0,inBallRadius+2*tol1-grooveDepth), Vector3d(0,0, inBallRadius+2*tol1), pinRadius-tol2, N);
    suppMesh = meshBoolean.MeshConnect(suppMesh, pin1);
    pin1 ->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI,0)));
    suppMesh = meshBoolean.MeshUnion(suppMesh, pin1);
    // touch
    partB_touch = Vector3d(0,0, -inBallRadius-lengthCube/2);
    Mesh* touchSupp = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0, -lengthCube/2 + tol2), rl , N);
    Mesh* touchSuppSPh = meshCreator.CreateSphere(rl, partB_touch, M_size, M_size);
    touchSupp = meshBoolean.MeshConnect(touchSupp, touchSuppSPh);
    suppMesh = meshBoolean.MeshUnion(suppMesh, touchSupp);
    cout<<"finish U joint outside supp"<<endl;

    /// spin around \beta = (0, 0, 1) in [0,2Pi]; rotate \alpha = (0,1,0) in [-Pi/3. Pi/3], M = R_\alpha * R_\beta
    Matrix4d rM = GetRotationMatrix(Vector3d(M_PI_2, 0, 0));
    ballMesh->TransformMesh(rM);
    suppMesh->TransformMesh(rM);
    MultiplyPoint(partA_touch, rM, partA_touch);
    MultiplyPoint(partB_touch, rM, partB_touch);
    partA = ballMesh;
    partB = suppMesh;
}

void Joint::CreateSimpleModel(const Vector3d &touchPA, const Vector3d &touchPB)
{
    switch (jointType)
    {
        case S_joint:
            CreateSimpleS(touchPA, touchPB);
            break;
        case R_joint:
            CreateSimpleR(touchPA, touchPB);
            break;
        case P_joint:
            CreateSimpleP(touchPA, touchPB);
            break;
        case U_joint:
            CreateSimpleU(touchPA, touchPB);
            break;
        case C_joint:
            CreateSimpleC(touchPA, touchPB);
            break;

        default:
            break;
    }
    //MeshBoolean meshBoolean;
    //partAll = meshBoolean.MeshConnect(partA,  partB);
}

void Joint::CreateSimpleS(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6; //0.6
    double ball_out_radius = 10.0*scale;
    double ball_in_radius = 7.25*scale;
    double tol = 0.25*scale;
    double radius_cyl = 3.5*scale;

    /// inside ball (1,0,0)
    double r_small = ball_in_radius;
    Mesh* s_in = meshCreator.CreateSphere(r_small - tol, N_size, N_size);
    partA_touch = touchPA;
    Mesh* s_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_cyl, N_size);
    Mesh* s_in_touch_sph = meshCreator.CreateSphere(radius_cyl, partA_touch, M_size, M_size);
    s_in_touch = meshBoolean.MeshConnect(s_in_touch, s_in_touch_sph);
    partA = meshBoolean.MeshUnion(s_in, s_in_touch);
    //cout<<"finish in ball"<<endl;

    /// outside ball (1, 0, 0)
    double r_big = ball_out_radius;
    Mesh* s_out = meshCreator.CreateSphere(r_big, N_size, N_size);
    partB_touch = Vector3d(-1,0,0) + Vector3d (-0.1, sqrt(0.99), 0) + Vector3d(-0.1, 0, sqrt(0.99));
    partB_touch = touchPB.normalized()* 2*r_small;
    Mesh* touchM_b = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, radius_cyl, N_size);
    Mesh* touchM_bSph = meshCreator.CreateSphere(radius_cyl, partB_touch, M_size,M_size);
    touchM_b = meshBoolean.MeshConnect(touchM_b, touchM_bSph);
    s_out = meshBoolean.MeshConnect(s_out, touchM_b);
    Mesh* s_out_in = meshCreator.CreateSphere(r_small + tol, N_size , N_size);
    s_out = meshBoolean.MeshMinus(s_out, s_out_in);
    //cout<<"finish out ball"<<endl;

    //hole (axis, range) ~ 1
    vector<Vector3d> axis_ns;
    vector<double> ranges;
    axis_ns.emplace_back(1,0,0);
    ranges.emplace_back(M_PI*1.01); //0.82

    for(int i=0;i<1;i++)
    {
        Vector3d axis_n = axis_ns[i];
        double rangeAngle = ranges[i];
        Mesh *cone = meshCreator.CreateCone(Vector3d(0, 0, 0), axis_n, ball_out_radius * 1.1, rangeAngle, N_size);
        double r_open, dis_open;
        r_open = (tol + ball_in_radius) * sin(rangeAngle / 2);
        dis_open = sqrt(pow(tol + ball_in_radius, 2) - pow(r_open, 2)) - sqrt(pow(ball_in_radius, 2) - pow(r_open, 2));
        //cout << "r_open = " << r_open << " dis_open = " << dis_open << endl;
        s_out = meshBoolean.MeshMinus(s_out, cone);
    }
    cout<<"finish simple s"<<endl;

    partB = s_out;
}

void Joint::CreateSimpleR(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6; //0.6

    double Radius_large_cyl = 10* scale;
    double tol1 = 0.25 * scale;
    double tol2 = 0.25 * scale;
    double thick  = 2.5 *scale;
    double Radius_small_cyl = Radius_large_cyl - thick - tol2;
    double height_small = 7*scale;
    double height_hat = 2.5*scale;
    double height_large = height_small + 2*tol1 + 2*height_hat;
    double radius_fol = 3.5*scale;

    double wh_out = 1.5*scale;

    int N = N_size;
    double hc = height_small;
    double tc = height_hat;

    /// cyl in & touch
    partA_touch = touchPA;
    Mesh* cyl_in = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tc ,0), Vector3d (0,hc/2+tc , 0), Radius_small_cyl, N);
    Mesh* cyl_in_h1 = meshCreator.CreateCylinder(Vector3d(0,hc/2+tol1,0), Vector3d (0,hc/2+tol1+tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_h2 = meshCreator.CreateCylinder(Vector3d(0,-hc/2-tol1,0), Vector3d (0,-hc/2-tol1-tc, 0),Radius_large_cyl, N);
    Mesh* cyl_in_touch = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch, radius_fol, N);
    Mesh* cyl_in_touch_sph = meshCreator.CreateSphere(radius_fol, partA_touch, M_size, M_size);
    cyl_in_touch = meshBoolean.MeshConnect(cyl_in_touch, cyl_in_touch_sph);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_touch);
    cyl_in = meshBoolean.MeshConnect(cyl_in, cyl_in_h1);
    cyl_in = meshBoolean.MeshUnion(cyl_in, cyl_in_h2);
    //cout<<"finish create R joint inside"<<endl;

    /// cyl out & touch
    partB_touch = touchPB;
    Mesh* cyl_out = meshCreator.CreateCylinder(Vector3d(0,-hc/2,0), Vector3d (0,hc/2, 0),Radius_large_cyl, N);
    Mesh* cyl_out_touch = meshCreator.CreateCylinder(Vector3d (0, 0,0),partB_touch, radius_fol,N);
    Mesh* cyl_out_touch_sph = meshCreator.CreateSphere(radius_fol, partB_touch, M_size, M_size);
    cyl_out_touch = meshBoolean.MeshConnect(cyl_out_touch, cyl_out_touch_sph);
    cyl_out = meshBoolean.MeshConnect(cyl_out,cyl_out_touch);
    Mesh* cyl_out_in = meshCreator.CreateCylinder(Vector3d(0,-hc,0), Vector3d (0,hc, 0),Radius_large_cyl-thick, N);
    cyl_out = meshBoolean.MeshMinus(cyl_out, cyl_out_in);
   // cout<<"finish create R joint outside"<<endl;

    partA = cyl_in;
    partB = cyl_out;
    cout<<"finish simple R"<<endl;

}

void Joint::CreateSimpleP(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;
    Vector3d SliderSize = Vector3d (5, 8, 8) * scale;
    double lengT = 50; //15 + 35
    double lengY, lengZ;
    double tol = 0.25*scale;
    double rl = 3.0*scale;
    lengY = lengZ = 3*scale;

    partA = meshCreator.CreateCuboid(Vector3d (-35 ,-lengY,-lengZ), Vector3d (35 ,lengY,lengZ));
    Mesh* addM = meshCreator.CreateCylinder(Vector3d(-35 + lengY,0,0), Vector3d(-35 + lengY, 0 ,10), lengY, CSG_NUM_N);
    Mesh* addM2 = meshCreator.CreateSphere(lengY, Vector3d(-35 + lengY, 0 ,10),CSG_NUM_N, CSG_NUM_N);
    partA = meshBoolean.MeshConnect(partA, addM2);
    partA = meshBoolean.MeshConnect(partA, addM);
    partA_touch = Vector3d(35 - lengY,0,0);

    partB = meshCreator.CreateCuboid(-SliderSize, SliderSize);
    partB_touch = touchPB;
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partB_touch, rl ,N_size);
    Mesh* touchMsph = meshCreator.CreateSphere(rl ,partB_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchMsph);
    partB = meshBoolean.MeshConnect(partB, touchM);
    Mesh* cutM = meshCreator.CreateCuboid(Vector3d (-lengT,-lengY-tol,-lengZ-tol), Vector3d (lengT,lengY+tol,lengZ+tol));
    partB = meshBoolean.MeshMinus(partB, cutM);

    cout<<"finish simple P"<<endl;

}

void Joint::CreateSimpleC(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    MeshCreator meshCreator;
    MeshBoolean meshBoolean;

    double scale = 0.6;
    ///
    double rl = 3.0*scale;
    double Rl = 7.5*scale;
    double lengT = 50;
    double thick = 5*scale;
    double fr = 3.0*scale;
    double tol = 0.25*scale;

    //partA = meshCreator.CreateCylinder(Vector3d(0, -lengT ,0), Vector3d(0,lengT ,0), rl, N_size);
    partA = meshCreator.CreateCylinder(Vector3d(0, -35 ,0), Vector3d(0,38 ,0), rl, N_size);
    partA_touch = Vector3d (0, -35 + fr ,0);

    if(touchPB.z() == -10)
    {
        partA = meshCreator.CreateCylinder(Vector3d(0, -18 ,0), Vector3d(0,8 ,0), rl, N_size);
        partA_touch = Vector3d (0, -18 + fr ,0);
    }


    partB = meshCreator.CreateCylinder(Vector3d(0,-thick, 0), Vector3d(0,thick,0), Rl, N_size);
    partB_touch = touchPB;
    Mesh* touchB = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0,0), fr,N_size);
    Mesh* touchBsph = meshCreator.CreateSphere(fr,partB_touch, M_size, M_size);
    touchB = meshBoolean.MeshConnect(touchB, touchBsph);
    partB = meshBoolean.MeshConnect(partB, touchB);

    Mesh* cutM = meshCreator.CreateCylinder(Vector3d(0,-lengT, 0), Vector3d(0,lengT, 0 ), rl+tol, N_size);
    partB = meshBoolean.MeshMinus(partB, cutM);

    cout<<"finish simple P"<<endl;

}

void Joint::CreateSimpleU(const Vector3d &touchPA, const Vector3d &touchPB)
{
    partMid = nullptr;
    double scale = 0.6; //0.6
    double inBallRadius = 7.5*scale;
    double tol1 = 0.4 * scale;
    double tol2 = 0.25 * scale;
    double lengthCube = 20.0*scale;
    double leftCube = -4.2*scale;
    double rightCube = 4.2*scale;
    double grooveDepth = 3 *scale;
    double pinRadius = 1.5 *scale;
    double rl = 3.5*scale;

    MeshCreator meshCreator;
    MeshBoolean meshBoolean;
    int N = N_size;

    /// inside ball
    // fan lie on x-z plane, thick deps on y-axis
    Mesh* ballMesh = meshCreator.CreateSphere(inBallRadius, Vector3d(0,0,0), N, N);
    Mesh* FanL = meshCreator.CreateFan(inBallRadius+tol1, 2*pinRadius, -M_PI/1.5, M_PI/1.5, N);
    Mesh* FanS = meshCreator.CreateFan(inBallRadius - grooveDepth - tol1, 2*pinRadius+0.1, -M_PI/1.2, M_PI/1.2, N);
    FanL = meshBoolean.MeshMinus(FanL, FanS);
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    FanL->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI, 0)));
    ballMesh = meshBoolean.MeshMinus(ballMesh, FanL);
    // touch
    partA_touch = touchPA;
    Mesh* touchM = meshCreator.CreateCylinder(Vector3d(0,0,0), partA_touch,rl,N);
    Mesh* touchSph = meshCreator.CreateSphere(rl, partA_touch, M_size, M_size);
    touchM = meshBoolean.MeshConnect(touchM, touchSph);
    ballMesh = meshBoolean.MeshUnion(ballMesh, touchM);
    //cout<<"finish U joint inside ball"<<endl;

    /// out supp
    Mesh* suppMesh = meshCreator.CreateCuboid(Vector3d(-lengthCube/2, leftCube, -lengthCube/2), Vector3d(lengthCube/2, rightCube, lengthCube/2));
    // touch
    partB_touch = touchPB;
    Mesh* touchSupp = meshCreator.CreateCylinder(partB_touch, Vector3d(0,0, 0), rl , N);
    Mesh* touchSuppSPh = meshCreator.CreateSphere(rl, partB_touch, M_size, M_size);
    touchSupp = meshBoolean.MeshConnect(touchSupp, touchSuppSPh);
    suppMesh = meshBoolean.MeshUnion(suppMesh, touchSupp);

    Mesh* cutBall = meshCreator.CreateSphere(inBallRadius+tol1, Vector3d(0,0,0), N, N);
    suppMesh = meshBoolean.MeshMinus(suppMesh, cutBall);
    // hole x-axis
    //Mesh* holeM = meshCreator.CreateCuboid(Vector3d(-lengthCube, leftCube/2, -lengthCube/4), Vector3d(lengthCube, rightCube/2, lengthCube/4));
    //suppMesh = meshBoolean.MeshMinus(suppMesh, holeM);
    // pin z-axis
    Mesh* pin1 = meshCreator.CreateCylinder(Vector3d(0,0,inBallRadius+2*tol1-grooveDepth), Vector3d(0,0, inBallRadius+2*tol1), pinRadius-tol2, N);
    suppMesh = meshBoolean.MeshConnect(suppMesh, pin1);
    pin1 ->TransformMesh(GetRotationMatrix(Vector3d(0,M_PI,0)));
    suppMesh = meshBoolean.MeshUnion(suppMesh, pin1);


    /// spin around \beta = (0, 0, 1) in [0,2Pi]; rotate \alpha = (0,1,0) in [-Pi/3. Pi/3], M = R_\alpha * R_\beta
    Matrix4d rM = GetRotationMatrix(Vector3d(M_PI_2, 0, 0));
    ballMesh->TransformMesh(rM);
    suppMesh->TransformMesh(rM);
    MultiplyPoint(partA_touch, rM, partA_touch);
    MultiplyPoint(partB_touch, rM, partB_touch);
    partA = ballMesh;
    partB = suppMesh;
    cout<<"finish simple U"<<endl;
}



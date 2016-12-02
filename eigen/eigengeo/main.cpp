#include <iostream>
//#include <iostream>
//#include <cmath>
//using namespace std;
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;

int main()
{
    //cout << "Hello World!" << endl;
    //Matrix<double, 3, 3> A;               // Fixed rows and cols. Same as Matrix3d.
    //Vector3f x, y, z;                     // 3x1 float matrix.
    Matrix3d rotation_matrix = Matrix3d::Identity();
    AngleAxisd rotation_vector ( M_PI/4, Vector3d ( 0,0,1 ) );

    cout.precision(3);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;

    rotation_matrix = rotation_vector.toRotationMatrix();

    Vector3d v ( 1,0,0 );
    Vector3d v_rotated = rotation_vector * v;
    //AngleAxisd v_rotatedm ()
    //cout.precision(3);
    cout<<"(1,0,0) after rotation = \n"<<v_rotated<<endl;

    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = \n"<<v_rotated<<endl;

    Vector3d euler_angles = rotation_matrix.eulerAngles ( 0,1,2 );
    cout<<"roll pitch yaw = \n"<<euler_angles<<endl;

    Isometry3d T=Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate ( rotation_vector );                                     // 按照rotation_vector进行旋转
    T.pretranslate ( Vector3d ( 1,3,4 ) );                     // 沿X轴平移1，相当于把平移向量设成(1,0,0)
    cout << "Transform matrix = \n" << T.matrix() <<endl;

    Vector3d v_transformed = T*v;                              // 相当于R*v+t
    cout<<"v tranformed = \n"<<v_transformed<<endl;

    Quaterniond q = Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部

    q = Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;

    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = \n"<<v_rotated<<endl;

    return 0;
}


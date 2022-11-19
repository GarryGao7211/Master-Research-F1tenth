#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include <complex>

using namespace std;


void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}

int solve_deg2(double a, double b, double c, double & x1, double & x2)
{
  double delta = b * b - 4 * a * c;

  if (delta < 0) return 0;

  double inv_2a = 0.5 / a;

  if (delta == 0) {
    x1 = -b * inv_2a;
    x2 = x1;
    return 1;
  }

  double sqrt_delta = sqrt(delta);
  x1 = (-b + sqrt_delta) * inv_2a;
  x2 = (-b - sqrt_delta) * inv_2a;
  return 2;
}

/// Reference : Eric W. Weisstein. "Cubic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/CubicEquation.html
/// \return Number of real roots found.
int solve_deg3(double a, double b, double c, double d,
               double & x0, double & x1, double & x2)
{
  if (a == 0) {
    // Solve second order system
    if (b == 0) {
      // Solve first order system
      if (c == 0)
    return 0;

      x0 = -d / c;
      return 1;
    }

    x2 = 0;
    return solve_deg2(b, c, d, x0, x1);
  }

  // Calculate the normalized form x^3 + a2 * x^2 + a1 * x + a0 = 0
  double inv_a = 1. / a;
  double b_a = inv_a * b, b_a2 = b_a * b_a;
  double c_a = inv_a * c;
  double d_a = inv_a * d;

  // Solve the cubic equation
  double Q = (3 * c_a - b_a2) / 9;
  double R = (9 * b_a * c_a - 27 * d_a - 2 * b_a * b_a2) / 54;
  double Q3 = Q * Q * Q;
  double D = Q3 + R * R;
  double b_a_3 = (1. / 3.) * b_a;

  if (Q == 0) {
    if(R == 0) {
      x0 = x1 = x2 = - b_a_3;
      return 3;
    }
    else {
      x0 = pow(2 * R, 1 / 3.0) - b_a_3;
      return 1;
    }
  }

  if (D <= 0) {
    // Three real roots
    double theta = acos(R / sqrt(-Q3));
    double sqrt_Q = sqrt(-Q);
    x0 = 2 * sqrt_Q * cos(theta             / 3.0) - b_a_3;
    x1 = 2 * sqrt_Q * cos((theta + 2 * 3.1415)/ 3.0) - b_a_3;
    x2 = 2 * sqrt_Q * cos((theta + 4 * 3.1415)/ 3.0) - b_a_3;

    return 3;
  }

  // D > 0, only one real root
  double AD = pow(fabs(R) + sqrt(D), 1.0 / 3.0) * (R > 0 ? 1 : (R < 0 ? -1 : 0));
  double BD = (AD == 0) ? 0 : -Q / AD;

  // Calculate the only real root
  x0 = AD + BD - b_a_3;

  return 1;
}

/// Reference : Eric W. Weisstein. "Quartic Equation." From MathWorld--A Wolfram Web Resource.
/// http://mathworld.wolfram.com/QuarticEquation.html
/// \return Number of real roots found.
int solve_deg4(double a, double b, double c, double d, double e,
               double & x0, double & x1, double & x2, double & x3)
{
  if (a == 0) {
    x3 = 0;
    return solve_deg3(b, c, d, e, x0, x1, x2);
  }

  // Normalize coefficients
  double inv_a = 1. / a;
  b *= inv_a; c *= inv_a; d *= inv_a; e *= inv_a;
  double b2 = b * b, bc = b * c, b3 = b2 * b;

  // Solve resultant cubic
  double r0, r1, r2;
  int n = solve_deg3(1, -c, d * b - 4 * e, 4 * c * e - d * d - b2 * e, r0, r1, r2);
  if (n == 0) return 0;

  // Calculate R^2
  double R2 = 0.25 * b2 - c + r0, R;
  if (R2 < 0)
    return 0;

  R = sqrt(R2);
  double inv_R = 1. / R;

  int nb_real_roots = 0;

  // Calculate D^2 and E^2
  double D2, E2;
  if (R < 10E-12) {
    double temp = r0 * r0 - 4 * e;
    if (temp < 0)
      D2 = E2 = -1;
    else {
      double sqrt_temp = sqrt(temp);
      D2 = 0.75 * b2 - 2 * c + 2 * sqrt_temp;
      E2 = D2 - 4 * sqrt_temp;
    }
  }
  else {
    double u = 0.75 * b2 - 2 * c - R2,
      v = 0.25 * inv_R * (4 * bc - 8 * d - b3);
    D2 = u + v;
    E2 = u - v;
  }

  double b_4 = 0.25 * b, R_2 = 0.5 * R;
  if (D2 >= 0) {
    double D = sqrt(D2);
    nb_real_roots = 2;
    double D_2 = 0.5 * D;
    x0 = R_2 + D_2 - b_4;
    x1 = x0 - D;
  }

  // Calculate E^2
  if (E2 >= 0) {
    double E = sqrt(E2);
    double E_2 = 0.5 * E;
    if (nb_real_roots == 0) {
      x0 = - R_2 + E_2 - b_4;
      x1 = x0 - E;
      nb_real_roots = 2;
    }
    else {
      x2 = - R_2 + E_2 - b_4;
      x3 = x2 - E;
      nb_real_roots = 4;
    }
  }

  return nb_real_roots;
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c/a - b*b/(3*a*a);
  float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3)/2);

  complex<float> inside = sqrt(q*q/4 + p*p*p/27);

  complex<float> root;

  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);
    //std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0) { return root; }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8*a*c-3*b*b)/(8*a*a);
  float q = (b*b*b-4*a*b*c+8*a*a*d)/(8*a*a*a);
  float r = (-3*b*b*b*b+256*a*a*a*e-64*a*a*b*d+16*a*b*b*c)/(256*a*a*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);

  complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;

  vector<complex<float>> roots { root1, root2, root3, root4 };

  float max_real_root = 0.f;

  for (complex<float> root: roots) {
    if(root.imag()==0){
    max_real_root = max(max_real_root, root.real());
  }
  //ROS_INFO_STREAM("root: " << root1);
  //ROS_INFO_STREAM("root: " << root2);
  //ROS_INFO_STREAM("root: " << root3);
  //ROS_INFO_STREAM("root: " << root4);

  //std::cout<<"Max real root:" << max_real_root<<std::endl;

  return max_real_root;
}}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans) {
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input : corresponds : a struct vector of Correspondene struct object defined in correspond.
  // input : curr_trans : A Transform object refernece
  // output : update the curr_trans object. Being a call by reference function, Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

// You can change the number of iterations here. More the number of iterations, slower will be the convergence but more accurate will be the results. You need to find the right balance.
int number_iter = 1.0;

for(int i = 0; i<number_iter; i++){

  //fill in the values of the matrics
  Eigen::MatrixXf M_i(2, 4);
  Eigen::Matrix2f C_i;
  Eigen::Vector2f pi_i;
  
  // Fill in the values for the matrices
  Eigen::Matrix4f M, W;
  Eigen::MatrixXf g(4, 1);
  M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  g << 0, 0, 0, 0;
  
  //ROS_INFO_STREAM("check: " << corresponds.size());
  for (int j = 0; j< corresponds.size(); j++) {
    //fill pi_i vector
    pi_i = corresponds[j].getPiVec();
    if(isnan(pi_i[0]) || isnan(pi_i[1])){
      continue;
    }
    //ROS_INFO_STREAM("pi_i[0]: " << pi_i[0]);
    //ROS_INFO_STREAM("pi_i[1]: " << pi_i[1]);
    M_i << 1, 0, corresponds[j].po->getX(), -corresponds[j].po->getY(), 0, 1, corresponds[j].po->getY(), corresponds[j].po->getX(); // fill M_i matrix
    //ROS_INFO_STREAM("Mi: " << M_i);

    C_i = corresponds[j].getNormalNorm()*corresponds[j].getNormalNorm().transpose(); // fill C_i matrix

    M = M + M_i.transpose()*C_i*M_i;
    //ROS_INFO_STREAM("M: " << M);
    //pi_i << corresponds[j].pj1->getX(), corresponds[j].pj2->getY();

    g = g - 2.0*(pi_i.transpose()*C_i*M_i).transpose();
    //ROS_INFO_STREAM("g: " << g);
  }

   //ROS_INFO_STREAM("indexi: " << i);
   //ROS_INFO_STREAM("M: " << M);
   //ROS_INFO_STREAM("g: " << g);
  // Define sub-matrices A, B, D from M
  Eigen::Matrix2f A, B, D;
  //Eigen::Matrix4f Two_M;
  //Two_M = 2.0 * M;
  M = M*2;

  //ROS_INFO_STREAM("2M: " << Two_M);
  A << M(0,0), M(0,1), M(1,0), M(1,1);
  //ROS_INFO_STREAM("!!!!!i: " << i);
  //ROS_INFO_STREAM("A: " << A);
  B << M(0,2), M(0,3), M(1,2), M(1,3);
  //ROS_INFO_STREAM("B: " << B);
  //ROS_INFO_STREAM("B_t: " << B.transpose());
  D << M(2,2), M(2,3), M(3,2), M(3,3);
  //ROS_INFO_STREAM("D: " << D);


  //define S and S_A matrices from the matrices A B and D
  Eigen::Matrix2f S;
  Eigen::Matrix2f S_A;

  //calculate S
  S = D - B.transpose()*A.inverse()*B;
  S_A = S.determinant()*S.inverse();


  //find the coefficients of the quadratic function of lambda
  float pow_2; float pow_1; float pow_0;
  pow_2 = 4.0;
  pow_1 = 2.0*(S(0,0) + S(1,1));
  pow_0 = S(0,0) * S(1,1) - (S(0,1)*S(1,0));


  // find the value of lambda by solving the equation formed. You can use the greatest real root function
  //double lambda;
  float a4; // 4th order coef on the left equation
  float b3; // 3th order coef 
  float c2; // 2nd order coef
  float d1; // 1sr order coef
  float e0; // constant
  a4 = pow_2*pow_2;
  b3 = 2*pow_2*pow_1;
  c2 = (2*pow_2*pow_0+(pow_1*pow_1));
  d1 = 2*pow_1*pow_0;
  e0 = pow_0*pow_0;


  // form first lambda2 block
  Eigen::Matrix4f lambda2_block;
  Eigen::Matrix2f eye2;
  eye2 << 1, 0, 0, 1;
  lambda2_block.block(0,0,2,2) = A.inverse()*B*B.transpose()*A.transpose().inverse();
  lambda2_block.block(0,2,2,2) = -A.inverse()*B;
  lambda2_block.block(2,0,2,2) = -B.transpose()*A.inverse().transpose();
  lambda2_block.block(2,2,2,2) = eye2;

  //form lambda1 block 
  Eigen::Matrix4f lambda1_block;
  lambda1_block.block(0,0,2,2) = A.inverse()*B*S_A*B.transpose()*A.transpose().inverse();
  lambda1_block.block(0,2,2,2) = -A.inverse()*B*S_A;
  lambda1_block.block(2,0,2,2) = -S_A.transpose()*B.transpose()*A.inverse().transpose();
  lambda1_block.block(2,2,2,2) = S_A;

  //form lambda block
  Eigen::Matrix4f lambda_block;
  lambda_block.block(0,0,2,2) = A.inverse()*B*S_A.transpose()*S_A*B.transpose()*A.transpose().inverse();
  lambda_block.block(0,2,2,2) = -A.inverse()*B*S_A.transpose()*S_A;
  lambda_block.block(2,0,2,2) = -1*S_A.transpose()*S_A*B.transpose()*A.inverse().transpose();
  lambda_block.block(2,2,2,2) = S_A.transpose()*S_A;

  // form poly on the right side
  Eigen::MatrixXf A2;
  Eigen::MatrixXf B1;
  Eigen::MatrixXf C0;
  float A20;
  float B10;
  float C00;
  A2 = 4.0*g.transpose()*lambda2_block*g;
  B1 = 4.0*g.transpose()*lambda1_block*g;
  C0 = g.transpose()*lambda_block*g;
  A20 = A2(0,0);
  B10 = B1(0,0);
  C00 = C0(0,0);


    //find the coefficients of the quaratic function of lambda
  double x1=0, x2=0, x3=0, x4=0;
  //float pow_4,pow_3,pow_2,pow_1,pow_0;
  int nbroots = solve_deg4(a4,b3,c2-A20,d1-B10,e0-C00,x1,x2,x3,x4);

  // find the value of lambda by solving the equation formed. You can use the greatest real root function
  double lambda = x1>x2?x1:x2;
  lambda = lambda>x3?lambda:x3;
  lambda = lambda>x4?lambda:x4;

  //lambda = greatest_real_root(a4,b3,c2-A20,d1-B10,e0-C00);
  ROS_INFO_STREAM("lambda: " << lambda);
  
  //find the value of x which is the vector for translation and rotation
  Eigen::Vector4f x;
  
  x = -(2.0*M+2.0*lambda*W).transpose().inverse()*g;

  // Convert from x to new transform

  float theta = atan2(x(3), x(2));
  curr_trans= Transform(x(0), x(1), theta);
  }

 


}

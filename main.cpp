#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <iostream>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>

#define VIENNACL_WITH_EIGEN 1

#include <Eigen/Core>
#include <Eigen/Dense>

#include <viennacl/vector.hpp>
#include <viennacl/matrix.hpp>
#include <viennacl/linalg/lu.hpp>

using namespace Eigen;
using namespace viennacl;

int main() {
  std::srand((unsigned int) time(0));
  
  const int n = 500;
  const float dt = 0.01;
  const float L = 1000.;
  
  // Position-Velocity Container 
  MatrixXf X = MatrixXf::Random(2*n,2);
  // initial positions
  float r = L/2.;
  for (int i = 0; i < n; i++) {
    X(i,0) = L/2 + r*std::cos(2 * 3.1415 * i / n);
    X(i,1) = L/2 + r*std::sin(2 * 3.1415 * i / n);

    X(2*i,0) *= 2.;
    X(2*i,1) *= 2.;
  }
  matrix<float> vcl_B(n,2);
  
  // Build problem matrix
  matrix<float> vcl_K(n,n);
  MatrixXf K = MatrixXf::Zero(n,n);
  {
    
    MatrixXf KK =  MatrixXf::Random(n,n); 
    KK = 0.5 * (KK + KK.transpose()).cwiseAbs(); // symmetrize
    KK.diagonal() = VectorXf::Zero(n);
    KK.diagonal() = - KK * VectorXf::Ones(n);
    K =  MatrixXf::Identity(n,n) - dt*dt*KK;
    
    // LU factorization on GPU
    copy(K, vcl_K);
    std::cout << "[GPU] LU factorization...";
    linalg::lu_factorize(vcl_K);
    std::cout << "done.\n";

    // LU factorization on CPU
    // std::cout << "[CPU] LU factorization...";
    // PartialPivLU<Matrix<float, n,n> > lu(K);
    //  std::cout << "done.\n";
  }

  // Plot
  sf::View view(sf::FloatRect(0., 0., L, L));
  sf::RenderWindow window(sf::VideoMode(L,L), "Plot");
  window.setFramerateLimit(30); // FPS limiter
  window.setView(view); //

  while (true) {
    window.clear(sf::Color(0.,0.,0.));

    MatrixXf Xold = X;
    
    // draw circles
    for (int i = 0; i < n; i++) {
      float x = Xold(i,0);
      float y = Xold(i,1);
      
      sf::CircleShape circle(1);
      circle.setPosition(sf::Vector2f(x,L-y));
      circle.setFillColor(sf::Color::Green);
      window.draw(circle);
    }

    MatrixXf B = Xold.block<n,2>(0,0) + dt * Xold.block<n,2>(n,0);
    
    // solve positions on CPU
    //X.block<n,2>(0,0) = lu.solve(B);

    // solve positions on GPU
    copy(B,vcl_B);
    linalg::lu_substitute(vcl_K, vcl_B);
    copy(vcl_B,B);
    X.block<n,2>(0,0) = B;
    
    
    // post process velocities
    X.block<n,2>(n,0) = (X.block<n,2>(0,0)-Xold.block<n,2>(0,0))/dt;
    
    //
    window.display();
  }

  window.close();
  
  return 0;
}

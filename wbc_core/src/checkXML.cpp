#include <jspace/test/sai_util.hpp>
#include <jspace/Model.hpp>
#include <err.h>
#include <signal.h>
#include <boost/scoped_ptr.hpp>

using namespace jspace;
using namespace boost;
using namespace std;

int main(int argc, char ** argv) {

  string robot_spec("trikey.xml");
  
  scoped_ptr<Model> model;

  try {
    static bool const enable_cc(false);
    model.reset(jspace::test::parse_sai_xml_file(robot_spec, enable_cc));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE,"exception parsing xml: %s", ee.what());
  }

  State state(9,9,6);
  state.position_ = Vector::Zero(9);
  state.velocity_ = Vector::Zero(9);

  model->update(state);

  Matrix A;
  model->getMassInertia(A);

  cout << "A:\n";
  for (size_t ii(0); ii<A.rows(); ++ii) {
    for (size_t jj(0); jj<A.cols(); ++jj) {
      cout << A(ii,jj) << "       ";
    }
    cout << "\n";
  }

  cout << "\n";
  /* 
  Matrix J;
  for (size_t ii(0); ii< state.position_.rows(); ++ii) {
    model->computeJacobianCOM(ii, J);
    cout << "J" << ii << ":\n";
    for (size_t jj(0); jj < 6; ++jj) {
      for (size_t kk(0); kk< state.position_.rows(); ++kk) {
	cout << J(jj,kk) << "      ";
      }
      cout <<"\n";
    }
    cout << "\n\n";
    }*/
  

}
